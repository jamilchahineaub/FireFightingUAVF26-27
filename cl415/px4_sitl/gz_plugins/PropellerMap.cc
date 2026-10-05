// propeller driven by motor+prop maps instead of the static k w^2 of MulticopterMotorModel.
// thrust, shaft torque and rpm per motor come from csv maps (airspeed down the rows, throttle across),
// made by flight_params/propulsion/prop_tools.py from APC data and a dc motor model.
// the command is the same gz::msgs::Actuators px4 sends to MulticopterMotorModel, read as
// throttle = velocity[motor_number] / command_full_throttle.
//
//   <plugin filename="libCl415PropellerMap.so" name="cl415::PropellerMap">
//     <joint_name>left_propeller_joint</joint_name>
//     <command_topic>/cl415/command/motor_speed</command_topic>
//     <motor_number>0</motor_number>
//     <command_full_throttle>1000</command_full_throttle>
//     <thrust_map>model://cl415_px4/maps/thrust_N.csv</thrust_map>   or relative to the sdf file
//     <torque_map>model://cl415_px4/maps/torque_Nm.csv</torque_map>
//     <rpm_map>model://cl415_px4/maps/rpm.csv</rpm_map>
//     <time_constant_up>0.05</time_constant_up>
//     <time_constant_down>0.08</time_constant_down>
//     <turning_direction>cw</turning_direction>      cw seen from behind = spin along +thrust axis
//     <visual_slowdown>10</visual_slowdown>
//   </plugin>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Util.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace cl415
{
// bilinear table: rows = airspeed, cols = throttle
struct Map2D
{
  std::vector<double> v, d;
  std::vector<std::vector<double>> z;

  bool Load(const std::string &_path)
  {
    std::ifstream f(_path);
    if (!f)
      return false;
    std::string line;
    std::getline(f, line);
    {
      std::stringstream ss(line);
      std::string cell;
      std::getline(ss, cell, ',');          // corner label
      while (std::getline(ss, cell, ','))
        this->d.push_back(std::stod(cell));
    }
    while (std::getline(f, line))
    {
      if (line.empty())
        continue;
      std::stringstream ss(line);
      std::string cell;
      std::getline(ss, cell, ',');
      this->v.push_back(std::stod(cell));
      std::vector<double> row;
      while (std::getline(ss, cell, ','))
        row.push_back(std::stod(cell));
      if (row.size() != this->d.size())
        return false;
      this->z.push_back(row);
    }
    return this->v.size() > 1 && this->d.size() > 1;
  }

  static void Bracket(const std::vector<double> &_x, double _q, size_t &_i, double &_w)
  {
    _q = std::clamp(_q, _x.front(), _x.back());
    _i = std::upper_bound(_x.begin(), _x.end(), _q) - _x.begin();
    _i = std::clamp<size_t>(_i, 1, _x.size() - 1);
    _w = (_q - _x[_i - 1]) / (_x[_i] - _x[_i - 1]);
  }

  double At(double _v, double _d) const
  {
    size_t i, j;
    double wv, wd;
    Bracket(this->v, _v, i, wv);
    Bracket(this->d, _d, j, wd);
    const double a = this->z[i - 1][j - 1] * (1 - wd) + this->z[i - 1][j] * wd;
    const double b = this->z[i][j - 1] * (1 - wd) + this->z[i][j] * wd;
    return a * (1 - wv) + b * wv;
  }
};

class PropellerMap : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->jointName = _sdf->Get<std::string>("joint_name", "").first;
    this->topic = _sdf->Get<std::string>("command_topic", "").first;
    this->motorNumber = _sdf->Get<int>("motor_number", 0).first;
    this->cmdFull = _sdf->Get<double>("command_full_throttle", 1000.0).first;
    this->tauUp = _sdf->Get<double>("time_constant_up", 0.05).first;
    this->tauDown = _sdf->Get<double>("time_constant_down", 0.08).first;
    this->slowdown = _sdf->Get<double>("visual_slowdown", 10.0).first;
    const auto dir = _sdf->Get<std::string>("turning_direction", "cw").first;
    this->spin = (dir == "ccw") ? -1.0 : 1.0;

    // model://name/maps/x.csv through the gz resource path, else relative to the sdf file
    std::string base = _sdf->FilePath();
    base = base.substr(0, base.find_last_of('/') + 1);
    auto resolve = [&](const std::string &_p) -> std::string
    {
      if (_p.rfind("model://", 0) == 0)
        return gz::common::findFile(_p);
      return (_p.empty() || _p[0] == '/') ? _p : base + _p;
    };
    const std::string tp = resolve(_sdf->Get<std::string>("thrust_map", "").first);
    const std::string qp = resolve(_sdf->Get<std::string>("torque_map", "").first);
    const std::string rp = resolve(_sdf->Get<std::string>("rpm_map", "").first);
    if (!this->thrust.Load(tp) || !this->torque.Load(qp) || !this->rpm.Load(rp))
    {
      gzerr << "[PropellerMap] could not read the maps: " << tp << ", " << qp << ", " << rp << "\n";
      return;
    }
    if (!this->node.Subscribe(this->topic, &PropellerMap::OnCommand, this))
    {
      gzerr << "[PropellerMap] could not subscribe to " << this->topic << "\n";
      return;
    }
    this->ok = true;
    gzmsg << "[PropellerMap] " << this->jointName << " motor " << this->motorNumber << " on " << this->topic
          << ", maps " << this->thrust.v.size() << "x" << this->thrust.d.size()
          << ", static full-throttle thrust " << this->thrust.At(0, 1) << " N\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->ok)
      return;
    if (this->jointEntity == gz::sim::kNullEntity && !this->Resolve(_ecm))
      return;
    if (_info.paused)
      return;

    const double dt = std::chrono::duration<double>(_info.dt).count();
    double ref;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      ref = this->throttleRef;
    }
    const double tau = ref > this->throttle ? this->tauUp : this->tauDown;
    this->throttle += (ref - this->throttle) * std::min(1.0, dt / std::max(tau, 1e-4));

    const auto propPose = this->propLink.WorldPose(_ecm);
    const auto basePose = this->baseLink.WorldPose(_ecm);
    const auto propVel = this->propLink.WorldLinearVelocity(_ecm);
    if (!propPose || !basePose || !propVel)
      return;

    // thrust axis: +z of the prop link (same as MulticopterMotorModel)
    const gz::math::Vector3d n = propPose->Rot().RotateVector(gz::math::Vector3d::UnitZ);
    gz::math::Vector3d vAir = *propVel;
    const auto windEntity = _ecm.EntityByComponents(gz::sim::components::Wind());
    if (windEntity != gz::sim::kNullEntity)
    {
      const auto wind = _ecm.Component<gz::sim::components::WorldLinearVelocity>(windEntity);
      if (wind)
        vAir -= wind->Data();
    }
    const double vAxial = std::max(0.0, vAir.Dot(n));

    // below 0.5 % throttle the motor is idle (px4 sends 1/1000 when armed so its esc check sees it online)
    const bool on = this->throttle > 5e-3;
    const double t = on ? this->thrust.At(vAxial, this->throttle) : 0.0;
    const double q = on ? this->torque.At(vAxial, this->throttle) : 0.0;
    const double w = on ? this->rpm.At(vAxial, this->throttle) * M_PI / 30.0 : 0.0;

    // thrust at the hub, prop reaction torque on the airframe, all applied to the parent link
    const gz::math::Vector3d force = n * t;
    const gz::math::Vector3d reaction = -this->spin * q * n;
    const gz::math::Vector3d arm = propPose->Pos() - basePose->Pos();
    this->baseLink.AddWorldWrench(_ecm, force, reaction + arm.Cross(force));

    // spin the visual prop (slowed down so the physics step can follow it)
    auto cmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(this->jointEntity);
    const double wv = this->spin * w / this->slowdown;
    if (cmd)
      cmd->Data() = {wv};
    else
      _ecm.CreateComponent(this->jointEntity, gz::sim::components::JointVelocityCmd({wv}));
  }

private:
  bool Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    this->jointEntity = this->model.JointByName(_ecm, this->jointName);
    if (this->jointEntity == gz::sim::kNullEntity)
      return false;
    gz::sim::Joint joint(this->jointEntity);
    const auto parent = joint.ParentLinkName(_ecm);
    const auto child = joint.ChildLinkName(_ecm);
    if (!parent || !child)
    {
      this->jointEntity = gz::sim::kNullEntity;
      return false;
    }
    this->baseLink = gz::sim::Link(this->model.LinkByName(_ecm, *parent));
    this->propLink = gz::sim::Link(this->model.LinkByName(_ecm, *child));
    this->propLink.EnableVelocityChecks(_ecm, true);
    return true;
  }

  void OnCommand(const gz::msgs::Actuators &_msg)
  {
    if (this->motorNumber >= _msg.velocity_size())
      return;
    std::lock_guard<std::mutex> lock(this->mutex);
    this->throttleRef = std::clamp(_msg.velocity(this->motorNumber) / this->cmdFull, 0.0, 1.0);
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};
  gz::sim::Link baseLink{gz::sim::kNullEntity};
  gz::sim::Link propLink{gz::sim::kNullEntity};
  gz::transport::Node node;
  std::mutex mutex;
  Map2D thrust, torque, rpm;
  std::string jointName, topic;
  int motorNumber{0};
  double cmdFull{1000.0}, tauUp{0.05}, tauDown{0.08}, slowdown{10.0}, spin{1.0};
  double throttleRef{0.0}, throttle{0.0};
  bool ok{false};
};
}  // namespace cl415

GZ_ADD_PLUGIN(cl415::PropellerMap, gz::sim::System, cl415::PropellerMap::ISystemConfigure,
              cl415::PropellerMap::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(cl415::PropellerMap, "cl415::PropellerMap")
