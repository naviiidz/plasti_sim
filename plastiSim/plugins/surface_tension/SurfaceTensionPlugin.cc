#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>

namespace gazebo
{
class SurfaceTensionPlugin : public ModelPlugin
{
  physics::LinkPtr link;
  event::ConnectionPtr updateConnection;
  double surface_z = 0.0;
  double tension_coeff = 0.072;
  double particle_radius = 0.0025;
  double wave_height = 0.0;
  double damping_coeff = 0.001;

  // Wave parameters
  double wave_amplitude = 0.005;
  double wave_frequency = 1.0;
  double wave_wavelength = 1.0;

  // Horizontal motion parameters
  double stokes_coeff = 0.0;
  double wind_strength = 0.0;
  ignition::math::Vector3d wind_direction = {1, 0, 0};
  double vortex_strength = 0.0;
  ignition::math::Vector3d vortex_center = {0, 0, 0};

public:
    void Load(physics::ModelPtr model, sdf::ElementPtr _sdf) override
    {
        link = model->GetLink("particle");
        if (!link)
        {
            gzerr << "[SurfaceTensionPlugin] Link 'particle' not found!\n";
            return;
        }
        if (_sdf->HasElement("tension_coeff"))
            tension_coeff = _sdf->Get<double>("tension_coeff");
        if (_sdf->HasElement("particle_radius"))
            particle_radius = _sdf->Get<double>("particle_radius");
        if (_sdf->HasElement("damping_coeff"))
            damping_coeff = _sdf->Get<double>("damping_coeff");
        wave_height = 2.0 * particle_radius;
        if (_sdf->HasElement("wave_height"))
            wave_height = _sdf->Get<double>("wave_height");
        if (_sdf->HasElement("wave_frequency"))
            wave_frequency = _sdf->Get<double>("wave_frequency");
        if (_sdf->HasElement("wave_wavelength"))
            wave_wavelength = _sdf->Get<double>("wave_wavelength");
        if (_sdf->HasElement("wave_amplitude"))
            wave_amplitude = _sdf->Get<double>("wave_amplitude");

        // Horizontal motion parameters
        if (_sdf->HasElement("stokes_coeff"))
            stokes_coeff = _sdf->Get<double>("stokes_coeff");
        if (_sdf->HasElement("wind_strength"))
            wind_strength = _sdf->Get<double>("wind_strength");
        if (_sdf->HasElement("wind_direction"))
            wind_direction = _sdf->Get<ignition::math::Vector3d>("wind_direction");
        if (_sdf->HasElement("vortex_strength"))
            vortex_strength = _sdf->Get<double>("vortex_strength");
        if (_sdf->HasElement("vortex_center"))
            vortex_center = _sdf->Get<ignition::math::Vector3d>("vortex_center");

        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SurfaceTensionPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
        auto pos = link->WorldCoGPose().Pos();
        double v_z = link->WorldLinearVel().Z();
        double time = link->GetWorld()->SimTime().Double();

        // Simulate a traveling sine wave surface at the particle's x position
        double k = 2 * M_PI / wave_wavelength;
        double omega = 2 * M_PI * wave_frequency;
        double surface_wave = wave_height * sin(k * pos.X() - omega * time);

        double effective_surface_z = surface_z + surface_wave;
        double dz = pos.Z() - effective_surface_z;

        // Vertical surface tension force
        if (std::abs(dz) < (2 * particle_radius + wave_height))
        {
            double F = -tension_coeff * dz - damping_coeff * v_z;
            link->AddForce(ignition::math::Vector3d(0, 0, F));
        }

        // Horizontal: Stokes drift (wave-induced)
        if (stokes_coeff != 0.0)
        {
            double stokes_vx = wave_amplitude * omega * cos(k * pos.X() - omega * time);
            link->AddForce(ignition::math::Vector3d(stokes_coeff * stokes_vx, 0, 0));
        }

        // Horizontal: Wind
        if (wind_strength != 0.0)
        {
            link->AddForce(wind_strength * wind_direction);
        }

        // Horizontal: Vortex
        if (vortex_strength != 0.0)
        {
            ignition::math::Vector3d r = pos - vortex_center;
            ignition::math::Vector3d vortex_force(-r.Y(), r.X(), 0);
            if (r.Length() > 1e-6)
                vortex_force.Normalize();
            link->AddForce(vortex_strength * vortex_force);
        }
    }
};
GZ_REGISTER_MODEL_PLUGIN(SurfaceTensionPlugin)
}