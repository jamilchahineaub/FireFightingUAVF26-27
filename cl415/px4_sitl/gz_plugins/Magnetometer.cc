// magnetometer for px4 on gz harmonic. the harmonic magnetometer sensor reports the field in a frame px4's
// gz_bridge only undoes for rotations about the vertical axis (gz-sim pr 2460), so in a banked turn px4 sees a
// heading that wanders by tens of degrees and the ekf drifts. this publishes the body-frame field computed from
// the link's true attitude and a fixed earth field, on the topic and in the frame gz_bridge expects
// (bridge: px4.x = -msg.y, px4.y = -msg.x, px4.z = msg.z, values in gauss).
//
//   <plugin filename="libCl415Magnetometer.so" name="cl415::Magnetometer">
//     <link_name>base_link</link_name>
//     <sensor_name>magnetometer_sensor</sensor_name>     the topic px4 subscribes to is built from this
//     <field_ned_gauss>0.2174 0.0125 0.4304</field_ned_gauss>
//     <update_rate>100</update_rate>
//     <noise_stddev>0.002</noise_stddev>
//   </plugin>

#include <gz/msgs/magnetometer.pb.h>

#include <chrono>
#include <random>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace cl415
{
class Magnetometer : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->linkName = _sdf->Get<std::string>("link_name", "base_link").first;
    this->sensorName = _sdf->Get<std::string>("sensor_name", "magnetometer_sensor").first;
    this->fieldNed = _sdf->Get<gz::math::Vector3d>("field_ned_gauss", {0.2174, 0.0125, 0.4304}).first;
    this->period = 1.0 / _sdf->Get<double>("update_rate", 100.0).first;
    this->noise = std::normal_distribution<double>(0.0, _sdf->Get<double>("noise_stddev", 0.002).first);
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (this->link == gz::sim::kNullEntity)
    {
      this->link = this->model.LinkByName(_ecm, this->linkName);
      if (this->link == gz::sim::kNullEntity)
        return;
      const auto world = _ecm.Component<gz::sim::components::Name>(gz::sim::worldEntity(_ecm));
      const std::string topic = "/world/" + world->Data() + "/model/" + this->model.Name(_ecm) + "/link/" +
                                this->linkName + "/sensor/" + this->sensorName + "/magnetometer";
      this->pub = this->node.Advertise<gz::msgs::Magnetometer>(topic);
      gzmsg << "[Magnetometer] publishing " << topic << "\n";
    }
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (t - this->lastPub < this->period)
      return;
    this->lastPub = t;

    const auto pose = gz::sim::worldPose(this->link, _ecm);
    const gz::math::Vector3d enu(this->fieldNed.Y(), this->fieldNed.X(), -this->fieldNed.Z());
    const gz::math::Vector3d flu = pose.Rot().RotateVectorReverse(enu);
    const gz::math::Vector3d frd(flu.X(), -flu.Y(), -flu.Z());

    gz::msgs::Magnetometer msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(gz::msgs::Convert(_info.simTime));
    msg.mutable_field_tesla()->set_x(-frd.Y() + this->noise(this->rng));
    msg.mutable_field_tesla()->set_y(-frd.X() + this->noise(this->rng));
    msg.mutable_field_tesla()->set_z(frd.Z() + this->noise(this->rng));
    this->pub.Publish(msg);
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity link{gz::sim::kNullEntity};
  std::string linkName, sensorName;
  gz::math::Vector3d fieldNed;
  double period{0.01}, lastPub{-1.0};
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  std::mt19937 rng{42};
  std::normal_distribution<double> noise{0.0, 0.002};
};
}  // namespace cl415

GZ_ADD_PLUGIN(cl415::Magnetometer, gz::sim::System, cl415::Magnetometer::ISystemConfigure,
              cl415::Magnetometer::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(cl415::Magnetometer, "cl415::Magnetometer")
