#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <iostream>

namespace gazebo
{
  class FloatSinkPlugin : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;
    
    double object_density;
    double water_density;
    double water_surface_z;
    double gravity_force;
    bool initialized;

  public:
    FloatSinkPlugin() : initialized(false) {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;
      
      // Get parameters from SDF
      std::string linkName = _sdf->HasElement("link_name") ? 
                            _sdf->Get<std::string>("link_name") : 
                            this->model->GetLinks()[0]->GetName();
      
      this->link = this->model->GetLink(linkName);
      if (!this->link)
      {
        gzerr << "[FloatSinkPlugin] Link not found: " << linkName << std::endl;
        return;
      }

      // Get volume from SDF
      double volume = _sdf->HasElement("volume") ? _sdf->Get<double>("volume") : 0.0;
      if (volume <= 0.0)
      {
        gzerr << "[FloatSinkPlugin] Please specify a positive <volume> in the plugin SDF." << std::endl;
        return;
      }

      // Get water density (default: 1000 kg/m^3)
      this->water_density = _sdf->HasElement("water_density") ? 
                           _sdf->Get<double>("water_density") : 1000.0;

      // Get water surface level (default: z = 0)
      this->water_surface_z = _sdf->HasElement("water_surface_z") ? 
                             _sdf->Get<double>("water_surface_z") : 0.0;

      // Calculate object density
      double mass = this->link->GetInertial()->Mass();
      this->object_density = mass / volume;

      // Calculate gravity force (mg)
      this->gravity_force = mass * 9.81;

      std::cout << "[FloatSinkPlugin] Object density: " << this->object_density 
                << " kg/m^3, Water density: " << this->water_density << " kg/m^3" << std::endl;
      std::cout << "[FloatSinkPlugin] Water surface at z = " << this->water_surface_z << " m" << std::endl;

      if (this->object_density < this->water_density)
        std::cout << "[FloatSinkPlugin] Object will float when in water." << std::endl;
      else
        std::cout << "[FloatSinkPlugin] Object will sink." << std::endl;

      // Connect to the world update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&FloatSinkPlugin::OnUpdate, this));

      this->initialized = true;
    }

  private:
    void OnUpdate()
    {
      if (!this->initialized || !this->link)
        return;

      // Get object position
      ignition::math::Vector3d pos = this->link->WorldPose().Pos();
      
      // Check if object is above or below water surface
      if (pos.Z() > this->water_surface_z)
      {
        // Object is above water - apply gravity
        ignition::math::Vector3d gravity_vec(0, 0, -this->gravity_force);
        this->link->AddForce(gravity_vec);
      }
      else
      {
        // Object is in/under water - apply buoyancy and gravity
        if (this->object_density < this->water_density)
        {
          // Object should float - apply upward buoyancy force
          double buoyancy_force = this->water_density * 9.81 * this->GetSubmergedVolume();
          ignition::math::Vector3d net_force(0, 0, buoyancy_force - this->gravity_force);
          this->link->AddForce(net_force);
          
          // Add damping when floating to stabilize
          ignition::math::Vector3d vel = this->link->WorldLinearVel();
          ignition::math::Vector3d damping = vel * -0.5;
          this->link->AddForce(damping);
        }
        else
        {
          // Object should sink - apply reduced gravity (accounting for partial buoyancy)
          double buoyancy_force = this->water_density * 9.81 * this->GetSubmergedVolume();
          ignition::math::Vector3d net_force(0, 0, buoyancy_force - this->gravity_force);
          this->link->AddForce(net_force);
        }
      }
    }

    double GetSubmergedVolume()
    {
      // For simplicity, assume full volume when submerged
      // In a more complex implementation, you'd calculate partial submersion
      ignition::math::Vector3d pos = this->link->WorldPose().Pos();
      if (pos.Z() <= this->water_surface_z)
        return this->link->GetInertial()->Mass() / this->object_density;
      else
        return 0.0;
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(FloatSinkPlugin)
}
