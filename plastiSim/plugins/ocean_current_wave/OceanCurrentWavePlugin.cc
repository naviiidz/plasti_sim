#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <cmath>

namespace gazebo
{
  class OceanCurrentWavePlugin : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;
    common::Time lastUpdateTime;
    
    // Current parameters
    ignition::math::Vector3d current_direction;
    double current_strength;
    double current_depth_falloff;
    
    // Wave parameters
    double wave_amplitude;
    double wave_frequency;
    double wave_length;
    double wave_speed;
    double water_surface_z;
    
    // Time tracking for wave motion
    double simulation_time;
    bool initialized;

  public:
    OceanCurrentWavePlugin() : initialized(false), simulation_time(0.0) {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;
      
      // Get link name from SDF
      std::string linkName = _sdf->HasElement("link_name") ? 
                            _sdf->Get<std::string>("link_name") : 
                            this->model->GetLinks()[0]->GetName();
      
      this->link = this->model->GetLink(linkName);
      if (!this->link)
      {
        gzerr << "[OceanCurrentWavePlugin] Link not found: " << linkName << std::endl;
        return;
      }

      // Load current parameters
      if (_sdf->HasElement("current_direction"))
      {
        this->current_direction = _sdf->Get<ignition::math::Vector3d>("current_direction");
        this->current_direction.Normalize(); // Ensure unit vector
      }
      else
      {
        this->current_direction = ignition::math::Vector3d(1, 0, 0); // Default: East
      }
      
      this->current_strength = _sdf->HasElement("current_strength") ? 
                              _sdf->Get<double>("current_strength") : 0.1; // m/s
      
      this->current_depth_falloff = _sdf->HasElement("current_depth_falloff") ? 
                                   _sdf->Get<double>("current_depth_falloff") : 0.1;

      // Load wave parameters
      this->wave_amplitude = _sdf->HasElement("wave_amplitude") ? 
                            _sdf->Get<double>("wave_amplitude") : 0.5; // meters
      
      this->wave_frequency = _sdf->HasElement("wave_frequency") ? 
                            _sdf->Get<double>("wave_frequency") : 0.5; // Hz
      
      this->wave_length = _sdf->HasElement("wave_length") ? 
                         _sdf->Get<double>("wave_length") : 10.0; // meters
      
      this->water_surface_z = _sdf->HasElement("water_surface_z") ? 
                             _sdf->Get<double>("water_surface_z") : 0.0;

      // Calculate wave speed from frequency and wavelength
      this->wave_speed = this->wave_frequency * this->wave_length;

      std::cout << "[OceanCurrentWavePlugin] Loaded for link: " << linkName << std::endl;
      std::cout << "[OceanCurrentWavePlugin] Current: " << this->current_strength 
                << " m/s in direction (" << this->current_direction.X() 
                << ", " << this->current_direction.Y() << ", " << this->current_direction.Z() << ")" << std::endl;
      std::cout << "[OceanCurrentWavePlugin] Waves: amplitude=" << this->wave_amplitude 
                << "m, frequency=" << this->wave_frequency << "Hz, length=" << this->wave_length << "m" << std::endl;

      // Connect to world update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&OceanCurrentWavePlugin::OnUpdate, this));

      this->lastUpdateTime = this->model->GetWorld()->SimTime();
      this->initialized = true;
    }

  private:
    void OnUpdate()
    {
      if (!this->initialized || !this->link)
        return;

      // Get current simulation time
      common::Time currentTime = this->model->GetWorld()->SimTime();
      double dt = (currentTime - this->lastUpdateTime).Double();
      this->lastUpdateTime = currentTime;
      this->simulation_time += dt;

      // Get object position
      ignition::math::Vector3d pos = this->link->WorldPose().Pos();
      
      // Only apply forces if object is near or below water surface
      if (pos.Z() > this->water_surface_z + 1.0) // 1m above water surface
        return;

      // Apply BOTH current forces (horizontal) AND wave forces (vertical) simultaneously
      ignition::math::Vector3d total_force = ignition::math::Vector3d::Zero;
      
      // Get current forces
      ignition::math::Vector3d current_force = this->CalculateCurrentForces(pos);
      
      // Get wave forces
      ignition::math::Vector3d wave_force = this->CalculateWaveForces(pos);
      
      // Combine both forces
      total_force = current_force + wave_force;
      
      // Apply the combined force
      this->link->AddForce(total_force);
    }

    ignition::math::Vector3d CalculateCurrentForces(const ignition::math::Vector3d& pos)
    {
      // Calculate depth factor (current weakens with depth below surface)
      double depth_below_surface = this->water_surface_z - pos.Z();
      double depth_factor = 1.0;
      
      if (depth_below_surface > 0)
      {
        // Exponential decay with depth
        depth_factor = std::exp(-depth_below_surface * this->current_depth_falloff);
      }
      
      // Calculate current force
      double mass = this->link->GetInertial()->Mass();
      double current_force_magnitude = this->current_strength * mass * depth_factor * 10.0; // Increased multiplier
      
      // Apply horizontal current force
      ignition::math::Vector3d current_force = this->current_direction * current_force_magnitude;
      
      // Only apply horizontal components (zero out Z for current)
      current_force.Z() = 0;
      
      return current_force;
    }

    ignition::math::Vector3d CalculateWaveForces(const ignition::math::Vector3d& pos)
    {
      // Only apply wave motion if object is at or near water surface
      double distance_from_surface = std::abs(pos.Z() - this->water_surface_z);
      if (distance_from_surface > this->wave_amplitude * 3.0)
        return ignition::math::Vector3d::Zero;

      // Calculate wave phase based on position and time
      double wave_number = 2.0 * M_PI / this->wave_length;
      double angular_frequency = 2.0 * M_PI * this->wave_frequency;
      
      // Use X position for wave propagation (waves travel in X direction)
      double phase = wave_number * pos.X() - angular_frequency * this->simulation_time;
      
      // Calculate vertical wave displacement and velocity
      double wave_height = this->wave_amplitude * std::sin(phase);
      double wave_velocity = this->wave_amplitude * angular_frequency * std::cos(phase);
      
      // Target Z position based on wave
      double target_z = this->water_surface_z + wave_height;
      
      // Apply restoring force toward wave surface
      double z_error = target_z - pos.Z();
      double mass = this->link->GetInertial()->Mass();
      
      // Proportional force to move object to wave surface
      double wave_force_z = mass * 50.0 * z_error; // Stronger spring to wave surface
      
      // Add wave velocity component
      ignition::math::Vector3d current_vel = this->link->WorldLinearVel();
      double velocity_error = wave_velocity - current_vel.Z();
      wave_force_z += mass * 10.0 * velocity_error; // Stronger damped motion
      
      // Add horizontal orbital motion (realistic wave behavior)
      double orbital_velocity_x = this->wave_amplitude * angular_frequency * std::sin(phase) * 0.3;
      double orbital_force_x = mass * 5.0 * orbital_velocity_x;
      
      // Return combined wave forces (both vertical and horizontal orbital motion)
      return ignition::math::Vector3d(orbital_force_x, 0, wave_force_z);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(OceanCurrentWavePlugin)
}
