#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

class MicroplasticPlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
        this->model = _model;
        
        // Load parameters with defaults
        this->waterDensity = _sdf->Get<double>("water_density", 1025.0).first;
        this->waterDragCoeff = _sdf->Get<double>("water_drag_coeff", 1.2).first;
        this->airDensity = _sdf->Get<double>("air_density", 1.225).first;
        this->airDragCoeff = _sdf->Get<double>("air_drag_coeff", 0.6).first;
        this->surfaceTension = _sdf->Get<double>("surface_tension", 0.072).first;
        this->contactAngle = _sdf->Get<double>("contact_angle", 85.0).first * M_PI / 180.0;
        
        // Current and wind (world coordinates)
        this->currentVel = _sdf->Get<ignition::math::Vector3d>("current_velocity", 
                    ignition::math::Vector3d(0.1, 0, 0)).first;
        this->windVel = _sdf->Get<ignition::math::Vector3d>("wind_velocity", 
                    ignition::math::Vector3d(5, 0, 0)).first;

        // Biofilm growth parameters
        this->biofilmThickness = 0.0;
        this->biofilmGrowthRate = _sdf->Get<double>("biofilm_growth_rate", 1e-6).first; // kg/m²/s
        this->lastUpdateTime = 0.0;
        this->lastBiofilmUpdate = 0.0; // Initialize biofilm update time

        gzmsg << "Microplastic plugin loaded for: " << _model->GetName() << "\n"
              << "  Water drag coeff: " << this->waterDragCoeff << "\n"
              << "  Air drag coeff: " << this->airDragCoeff << "\n"
              << "  Contact angle: " << this->contactAngle * 180.0/M_PI << "°\n";

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MicroplasticPlugin::OnUpdate, this, std::placeholders::_1));
    }

    void OnUpdate(const common::UpdateInfo &_info) {
        physics::LinkPtr link = this->model->GetLink("particle");
        if (!link) return;

        // Calculate delta time
        double currentTime = _info.simTime.Double();
        double dt = currentTime - this->lastUpdateTime;
        this->lastUpdateTime = currentTime;

        // Get current state
        ignition::math::Pose3d pose = link->WorldPose();
        ignition::math::Vector3d vel = link->WorldLinearVel();
        double radius = 0.0025; // Match SDF

        // 1. Calculate submerged volume (capillary model)
        double h = pose.Pos().Z(); // Height relative to water surface
        double submergedVolume = CalculateSubmergedVolume(h, radius);

        // 2. Biofilm growth (time-dependent)
        if (currentTime > this->lastBiofilmUpdate + 1.0) {
            this->biofilmThickness += this->biofilmGrowthRate * dt;
            this->lastBiofilmUpdate = currentTime;
        }

        // 3. Apply forces
        ApplyHydrodynamicForces(link, vel, submergedVolume, radius);
    }

private:
    double CalculateSubmergedVolume(double h, double r) {
        // Capillary approximation for microplastics
        double theta = this->contactAngle;
        double capHeight = std::max(0.0, r * (1 - cos(theta)));
        
        if (h >= r) return 0.0; // Fully airborne
        if (h <= -r) return (4.0/3.0)*M_PI*r*r*r; // Fully submerged

        // Partially submerged (spherical cap + meniscus correction)
        double Vcap = (M_PI/3.0) * pow(r - h, 2) * (2*r + h);
        return Vcap + 0.1*capHeight; // Empirical meniscus term
    }

void ApplyHydrodynamicForces(physics::LinkPtr _link, 
                           const ignition::math::Vector3d &_vel,
                           double _submergedVolume,
                           double _radius) 
{
    // 1. Get accurate sphere parameters
    double totalVolume = (4.0/3.0)*M_PI*pow(_radius,3);
    double exposedRatio = 1.0 - (_submergedVolume/totalVolume);
    double area = M_PI * _radius * _radius;

    // 2. Buoyancy force (corrected formula)
    double buoyancyZ = waterDensity * _submergedVolume * 9.81;
    // Only apply buoyancy when particle is below water surface (h < 0)
    double h = _link->WorldPose().Pos().Z();
    if (h >= 0) {
        buoyancyZ = 0.0; // No buoyancy above water
    }
    ignition::math::Vector3d buoyancyForce(0, 0, buoyancyZ);

    // 3. Water drag (only applied to submerged portion)
    ignition::math::Vector3d waterRelVel = _vel - currentVel;
    double waterDragMag = 0.5 * waterDensity * waterDragCoeff * 
                        (_submergedVolume/totalVolume) * area * 
                        waterRelVel.Length();
    ignition::math::Vector3d waterDrag = -waterDragMag * waterRelVel.Normalize();

    // 4. Air drag (only applied to exposed portion)
    ignition::math::Vector3d airRelVel = _vel - windVel;
    double airDragMag = 0.5 * airDensity * airDragCoeff * 
                      exposedRatio * area * 
                      airRelVel.Length();
    ignition::math::Vector3d airDrag = -airDragMag * airRelVel.Normalize();

    // 5. Gravity force (when above water surface)
    double gravityForce = 0.0;
    if (h >= 0) {
        gravityForce = -_link->GetInertial()->Mass() * 9.81; // Negative Z (downward)
    }
    ignition::math::Vector3d gravity(0, 0, gravityForce);

    // 6. FINALLY APPLY FORCES
    _link->AddForce(buoyancyForce);
    _link->AddForce(waterDrag);
    _link->AddForce(airDrag);
    _link->AddForce(gravity);
}

    // Member variables
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    double waterDensity, waterDragCoeff;
    double airDensity, airDragCoeff;
    double surfaceTension, contactAngle;
    double biofilmThickness, biofilmGrowthRate;
    double lastUpdateTime, lastBiofilmUpdate;
    ignition::math::Vector3d currentVel, windVel;
    int debugCounter = 0;
};

GZ_REGISTER_MODEL_PLUGIN(MicroplasticPlugin)
} // namespace gazebo