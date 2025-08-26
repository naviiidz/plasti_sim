#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <random>
#include <cmath>

namespace gazebo
{
class SurfaceMotionPlugin : public ModelPlugin
{
  physics::LinkPtr link;
  double wind_strength = 0.0;
  ignition::math::Vector3d wind_direction = {1, 0, 0};
  double particle_radius = 0.0025; // Make sure to set this from SDF if needed
  double fluid_density = 1100.0;   // Set to your water density (kg/m^3)
  double dynamic_viscosity = 0.001; // Water at ~20C (Pa.s)

  // ... (other parameters and code) ...

public:
  void Load(physics::ModelPtr model, sdf::ElementPtr _sdf) override
  {
    link = model->GetLink("particle");
    if (_sdf->HasElement("wind_strength"))
      wind_strength = _sdf->Get<double>("wind_strength");
    if (_sdf->HasElement("wind_direction"))
      wind_direction = _sdf->Get<ignition::math::Vector3d>("wind_direction");
    if (_sdf->HasElement("particle_radius"))
      particle_radius = _sdf->Get<double>("particle_radius");
    if (_sdf->HasElement("fluid_density"))
      fluid_density = _sdf->Get<double>("fluid_density");
    if (_sdf->HasElement("dynamic_viscosity"))
      dynamic_viscosity = _sdf->Get<double>("dynamic_viscosity");

    updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SurfaceMotionPlugin::OnUpdate, this));
  }

  void OnUpdate()
  {
    if (!link) return;
    auto v = link->WorldLinearVel();
    double v_mag = v.Length();

    // Reynolds number
    double Re_p = (2 * particle_radius) * v_mag * fluid_density / dynamic_viscosity;

    // Drag coefficient
    double C_D = std::max(24.0 / Re_p * (1 + 0.15 * pow(Re_p, 0.687)), 0.44);

    // Drag force: Fd = 0.5 * C_D * rho * A * v^2 (opposite to velocity)
    double area = M_PI * particle_radius * particle_radius;
    ignition::math::Vector3d drag = -0.5 * C_D * fluid_density * area * v_mag * v;

    // Wind force
    ignition::math::Vector3d wind_force = wind_strength * wind_direction;

    // Apply forces
    link->AddForce(wind_force + drag);
  }

private:
  event::ConnectionPtr updateConnection;
};
GZ_REGISTER_MODEL_PLUGIN(SurfaceMotionPlugin)
}