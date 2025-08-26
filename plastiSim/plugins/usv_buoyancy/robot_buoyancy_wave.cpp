#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <vector>
#include <string>

namespace gazebo
{
    class RobotBuoyancyWave : public ModelPlugin
    {
    private:
        event::ConnectionPtr updateConnection;
        std::vector<physics::LinkPtr> links;
        std::vector<double> volumes;
        std::vector<double> radii;
        std::vector<double> lengths;
        double liquid_density = 1000.0;

    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
        {
            // Support multiple links via SDF
            sdf::ElementPtr linkElem = _sdf->GetElement("link");
            while (linkElem)
            {
                std::string linkName = linkElem->Get<std::string>("name");
                auto link = _model->GetLink(linkName);
                if (!link)
                {
                    std::cerr << "No link named '" << linkName << "' found in model.\n";
                }
                else
                {
                    links.push_back(link);
                    double volume = linkElem->HasElement("volume") ? linkElem->Get<double>("volume") : 0.0;
                    double radius = linkElem->HasElement("radius") ? linkElem->Get<double>("radius") : 0.25;
                    double length = linkElem->HasElement("cyl_height") ? linkElem->Get<double>("cyl_height") : 2.0;
                    volumes.push_back(volume);
                    radii.push_back(radius);
                    lengths.push_back(length);
                }
                linkElem = linkElem->GetNextElement("link");
            }
            if (_sdf->HasElement("liquid_density"))
                liquid_density = _sdf->Get<double>("liquid_density");

            updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RobotBuoyancyWave::OnUpdate, this));
        }

        // Compute buoyancy force at a given point along the cylinder
        double buoyancyForceAtPoint(double density, double radius, double z_point, double length = 2.0)
        {
            // Water surface at z=0
            double h = std::max(0.0, std::min(2 * radius, radius - z_point));
            double submerged_volume = 0.0;
            if (h > 0 && h < 2 * radius)
            {
                double theta = std::acos((radius - h) / radius);
                submerged_volume = (radius * radius * theta - (radius - h) * std::sqrt(2 * radius * h - h * h))*length/2;
            }
            else if (h >= 2 * radius)
            {
                submerged_volume = M_PI * radius * radius* length/2; // Full cylinder submerged
            }
            return density * submerged_volume * 9.81;
        }

        void OnUpdate()
        {
            // --- Stokes drift parameters ---
            double A = 0.2;      // Wave amplitude (meters)
            double omega = 1.0;  // Wave angular frequency (rad/s)
            double k = 2.0;      // Wave number (rad/m)
            double stokes_gain = 10.0; // Tune for effect strength

            // Get simulation time
            double t = links.empty() ? 0.0 : links[0]->GetWorld()->SimTime().Double();

            for (size_t i = 0; i < links.size(); ++i)
            {
                auto link = links[i];
                double radius = radii[i];
                double length = lengths[i];

                ignition::math::Pose3d pose = link->WorldPose();
                ignition::math::Vector3d center = pose.Pos();
                double half_length = length / 2.0;

                ignition::math::Vector3d local_end1(0, 0, half_length);
                ignition::math::Vector3d local_end2(0, 0, -half_length);

                ignition::math::Vector3d end1 = pose.CoordPositionAdd(local_end1);
                ignition::math::Vector3d end2 = pose.CoordPositionAdd(local_end2);

                double buoyancy_end1 = buoyancyForceAtPoint(liquid_density, radius, end1.Z());
                double buoyancy_end2 = buoyancyForceAtPoint(liquid_density, radius, end2.Z());

                if (buoyancy_end1 > 0.0)
                    link->AddForceAtWorldPosition(ignition::math::Vector3d(0, 0, buoyancy_end1), end1);
                if (buoyancy_end2 > 0.0)
                    link->AddForceAtWorldPosition(ignition::math::Vector3d(0, 0, buoyancy_end2), end2);



                // --- Add Stokes drift (wave-induced motion) ---
                // Stokes drift velocity at the surface: u_s = (omega * A^2 * k) * cos(omega * t)
                double stokes_vel = omega * A * A * k * std::cos(omega * t);
                double mass = link->GetInertial()->Mass();
                ignition::math::Vector3d stokesForce(stokes_gain * mass * stokes_vel, 0, 0);
                link->AddForce(stokesForce);
            }
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(RobotBuoyancyWave)
}