#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class ThrustPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->leftLink = model->GetLink("left_floater");
    this->rightLink = model->GetLink("right_floater");
    this->leftJoint = model->GetJoint(sdf->Get<std::string>("left_wheel_joint"));
    this->rightJoint = model->GetJoint(sdf->Get<std::string>("right_wheel_joint"));
    this->gain = sdf->HasElement("thrust_gain") ? sdf->Get<double>("thrust_gain") : 1000.0;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ThrustPlugin::OnUpdate, this));
  }

  void OnUpdate()
  {
    if (!this->leftLink || !this->rightLink || !this->leftJoint || !this->rightJoint)
      return;

    double leftVel = this->leftJoint->GetVelocity(0);
    double rightVel = this->rightJoint->GetVelocity(0);

    // Convert wheel velocity to force in Z direction
    ignition::math::Vector3d leftForce(0, 0, this->gain * leftVel);
    ignition::math::Vector3d rightForce(0, 0, this->gain * rightVel);

    this->leftLink->AddRelativeForce(leftForce);
    this->rightLink->AddRelativeForce(rightForce);
  }

private:
  physics::LinkPtr leftLink;
  physics::LinkPtr rightLink;
  physics::JointPtr leftJoint;
  physics::JointPtr rightJoint;
  double gain;
  event::ConnectionPtr updateConnection;
};
GZ_REGISTER_MODEL_PLUGIN(ThrustPlugin)
}