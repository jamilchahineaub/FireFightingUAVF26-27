// chase camera: moves a (static) camera model so it sits behind and above a target model, looking at it,
// following the target's heading but not its roll and pitch. put a camera sensor with the CameraVideoRecorder
// system on that model and the video is a smooth external chase view, with no gui involved.
//
//   <plugin filename="libCl415ChaseCamera.so" name="cl415::ChaseCamera">
//     <target>cl415</target>
//     <offset>-7 -3 2</offset>          back, right(-)/left(+), up, in the target's heading frame [m]
//     <look_height>0.3</look_height>    aim this much above the target origin [m]
//     <smoothing>0.15</smoothing>       first-order lag on the heading and position [s]
//   </plugin>

#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <sdf/Element.hh>

namespace cl415
{
class ChaseCamera : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->targetName = _sdf->Get<std::string>("target", "cl415").first;
    this->offset = _sdf->Get<gz::math::Vector3d>("offset", {-7, -3, 2}).first;
    this->lookHeight = _sdf->Get<double>("look_height", 0.3).first;
    this->tau = _sdf->Get<double>("smoothing", 0.15).first;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (this->target == gz::sim::kNullEntity)
    {
      this->target = _ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name(this->targetName));
      if (this->target == gz::sim::kNullEntity)
        return;
      gzmsg << "[ChaseCamera] following " << this->targetName << "\n";
    }
    const auto tp = gz::sim::worldPose(this->target, _ecm);
    const double yaw = tp.Rot().Yaw();
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const double a = this->tau > 1e-3 ? std::min(1.0, dt / this->tau) : 1.0;
    // unwrap the heading before filtering it
    double dyaw = yaw - this->yawF;
    while (dyaw > M_PI) dyaw -= 2 * M_PI;
    while (dyaw < -M_PI) dyaw += 2 * M_PI;
    this->yawF = this->init ? this->yawF + a * dyaw : yaw;
    const gz::math::Quaterniond rz(0, 0, this->yawF);
    const gz::math::Vector3d want = tp.Pos() + rz.RotateVector(this->offset);
    this->posF = this->init ? this->posF + (want - this->posF) * a : want;
    this->init = true;

    const gz::math::Vector3d aim = tp.Pos() + gz::math::Vector3d(0, 0, this->lookHeight);
    const gz::math::Vector3d d = aim - this->posF;
    const double camYaw = std::atan2(d.Y(), d.X());
    const double camPitch = -std::atan2(d.Z(), std::hypot(d.X(), d.Y()));
    const gz::math::Pose3d pose(this->posF, gz::math::Quaterniond(0, camPitch, camYaw));
    auto cmd = _ecm.Component<gz::sim::components::WorldPoseCmd>(this->model.Entity());
    if (cmd)
      cmd->Data() = pose;
    else
      _ecm.CreateComponent(this->model.Entity(), gz::sim::components::WorldPoseCmd(pose));
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity target{gz::sim::kNullEntity};
  std::string targetName;
  gz::math::Vector3d offset{-7, -3, 2}, posF{0, 0, 0};
  double lookHeight{0.3}, tau{0.15}, yawF{0};
  bool init{false};
};
}  // namespace cl415

GZ_ADD_PLUGIN(cl415::ChaseCamera, gz::sim::System, cl415::ChaseCamera::ISystemConfigure,
              cl415::ChaseCamera::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(cl415::ChaseCamera, "cl415::ChaseCamera")
