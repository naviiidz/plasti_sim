#include <rclcpp/rclcpp.hpp>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/transport/Node.hh>
#include <random>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

class DebrisSpawner : public rclcpp::Node
{
public:
    DebrisSpawner() : Node("debris_spawner")
    {
        // Initialize Gazebo transport node
        gz_node_ = std::make_shared<gz::transport::Node>();
        
        // Define spawn region bounds - Near WAM-V location in VRX Sydney Regatta world
        // WAM-V is at approximately [-532.467, 161.955, -0.147915]
        spawn_region_.min_x = -531.0;  // 10m west of WAM-V
        spawn_region_.max_x = -512.0;  // 10m east of WAM-V  
        spawn_region_.min_y = 156.0;   // 10m south of WAM-V
        spawn_region_.max_y = 250.0;   // 10m north of WAM-V
        spawn_region_.min_z = 1.0;     // Spawn 1m above water to let objects fall naturally
        spawn_region_.max_z = 2.0;     // Spawn 1-2m above water surface

        // Available plastic debris models - using installed model paths
        debris_models_ = {
            "plastic_bottle"
        };
        
        // Initialize random number generator
        rng_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
        
        // Parameters - more realistic now that ODE crash issue is resolved
        this->declare_parameter("num_objects", 10);      // Spawn 10 objects
        this->declare_parameter("spawn_interval", 5.0);  // Every 5 seconds
        this->declare_parameter("max_objects", 20);      // Maximum 20 objects total
        
        num_objects_to_spawn_ = this->get_parameter("num_objects").as_int();
        spawn_interval_ = this->get_parameter("spawn_interval").as_double();
        max_objects_ = this->get_parameter("max_objects").as_int();
        
        RCLCPP_INFO(this->get_logger(), "DebrisSpawner initialized");
        RCLCPP_INFO(this->get_logger(), "Spawn region: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]",
                    spawn_region_.min_x, spawn_region_.max_x,
                    spawn_region_.min_y, spawn_region_.max_y,
                    spawn_region_.min_z, spawn_region_.max_z);
        RCLCPP_INFO(this->get_logger(), "Will spawn %d objects with %.1f second intervals",
                    num_objects_to_spawn_, spawn_interval_);
        
        // Start spawning after a short delay
        spawn_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(spawn_interval_ * 1000)),
            std::bind(&DebrisSpawner::spawnDebris, this));
    }

private:
    struct SpawnRegion {
        double min_x, max_x;
        double min_y, max_y;
        double min_z, max_z;
    };
    
    void spawnDebris()
    {
        if (objects_spawned_ >= num_objects_to_spawn_ || total_objects_ >= max_objects_) {
            RCLCPP_INFO(this->get_logger(), "Spawning complete. Spawned %d objects total.", objects_spawned_);
            spawn_timer_->cancel();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting spawn attempt %d/%d", objects_spawned_ + 1, num_objects_to_spawn_);
        
        // Additional safety check - if we've had failures, slow down even more
        if (objects_spawned_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Previous spawn completed, adding extra safety delay...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 5 second extra delay between objects
        }
        
        // Generate random position within spawn region
        double x = getRandomDouble(spawn_region_.min_x, spawn_region_.max_x);
        double y = getRandomDouble(spawn_region_.min_y, spawn_region_.max_y);
        double z = getRandomDouble(spawn_region_.min_z, spawn_region_.max_z);
        
        // Safety bounds check to prevent ODE collision errors
        const double MAX_COORD = 600.0; // Increased to accommodate WAM-V area (around -540 to -520)
        if (std::abs(x) > MAX_COORD || std::abs(y) > MAX_COORD || std::abs(z) > MAX_COORD) {
            RCLCPP_ERROR(this->get_logger(), "Generated coordinates exceed safe bounds, skipping spawn");
            objects_spawned_++; // Skip this spawn
            return;
        }
        
        // Random orientation - NO rotation to prevent ODE collision errors
        double roll = 0.0;   // No roll to prevent collision errors
        double pitch = 0.0;  // No pitch to prevent collision errors  
        double yaw = 0.0;    // No yaw to prevent collision errors
        
        // Select random debris model
        const std::string& model_name = debris_models_[rng_() % debris_models_.size()];
        
        // Create unique entity name
        std::string entity_name = "debris_" + std::to_string(objects_spawned_) + "_" + std::to_string(getCurrentTimestamp());
        
        RCLCPP_INFO(this->get_logger(), "Attempting to spawn object %d/%d: %s (%s) at (%.1f, %.1f, %.1f)",
                    objects_spawned_ + 1, num_objects_to_spawn_, entity_name.c_str(), model_name.c_str(), x, y, z);
        
        // Create SDF for spawning
        std::string sdf_content = createSpawnSDF(model_name, entity_name, x, y, z, roll, pitch, yaw);
        
        // Send spawn request with retry logic
        int max_retries = 2; // Reduced retries to prevent long waits
        bool success = false;
        for (int retry = 0; retry < max_retries && !success; retry++) {
            if (retry > 0) {
                RCLCPP_WARN(this->get_logger(), "Retrying spawn request (attempt %d/%d)", retry + 1, max_retries);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // Wait 2 seconds between retries
            }
            
            success = sendSpawnRequest(sdf_content);
            
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Spawn request succeeded on attempt %d", retry + 1);
            } else {
                RCLCPP_WARN(this->get_logger(), "Spawn request failed on attempt %d", retry + 1);
            }
        }
        
        if (success) {
            objects_spawned_++;
            total_objects_++;
            RCLCPP_INFO(this->get_logger(), "Successfully spawned object %d/%d: %s (%s)",
                        objects_spawned_, num_objects_to_spawn_, entity_name.c_str(), model_name.c_str());
            
            // Give physics MUCH more time to stabilize after spawn to prevent ODE errors
            RCLCPP_INFO(this->get_logger(), "Allowing physics to stabilize (extended period)...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // 5 second stabilization
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn object after %d attempts: %s (%s)", 
                         max_retries, entity_name.c_str(), model_name.c_str());
            // Still increment counter to avoid infinite retries
            objects_spawned_++;
        }
        
        // Add extra delay after each spawn attempt to be safe
        RCLCPP_INFO(this->get_logger(), "Waiting for next spawn cycle...");
    }
    
    std::string createSpawnSDF(const std::string& model_name, const std::string& entity_name,
                              double x, double y, double z, double roll, double pitch, double yaw)
    {
        std::stringstream sdf_stream;
        sdf_stream << "<?xml version=\"1.0\" ?>\n"
                   << "<sdf version=\"1.7\">\n"
                   << "  <include>\n"
                   << "    <uri>model://plastiSim/models/" << model_name << "</uri>\n"
                   << "    <name>" << entity_name << "</name>\n"
                   << "    <pose>" << x << " " << y << " " << z << " " 
                   << roll << " " << pitch << " " << yaw << "</pose>\n"
                   << "  </include>\n"
                   << "</sdf>";
        return sdf_stream.str();
    }
    
    bool sendSpawnRequest(const std::string& sdf_content)
    {
        gz::msgs::EntityFactory request;
        gz::msgs::Boolean response;
        bool result = false;
        
        request.set_sdf(sdf_content);
        request.set_allow_renaming(true);
        
        RCLCPP_DEBUG(this->get_logger(), "Sending spawn request to Gazebo...");
        
        // Send request to Gazebo with longer timeout to prevent crashes
        const unsigned int timeout = 15000;  // Increased to 15 seconds
        bool executed = false;
        
        try {
            result = gz_node_->Request("/world/sydney_regatta/create", request, timeout, response, executed);
            
            RCLCPP_DEBUG(this->get_logger(), "Request result: %s, executed: %s", 
                        result ? "true" : "false", executed ? "true" : "false");
            
            if (!result) {
                RCLCPP_WARN(this->get_logger(), "Spawn request failed: service request failed");
                return false;
            }
            
            if (!executed) {
                RCLCPP_WARN(this->get_logger(), "Spawn request failed: request not executed (timeout)");
                return false;
            }
            
            if (!response.data()) {
                RCLCPP_WARN(this->get_logger(), "Spawn request rejected by Gazebo (response.data() = false)");
                return false;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Spawn request successful");
            
            // Add a longer delay after successful spawn to prevent overwhelming Gazebo
            RCLCPP_INFO(this->get_logger(), "Spawn request completed, allowing Gazebo to process...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1 second processing time
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during spawn request: %s", e.what());
            return false;
        }
    }
    
    double getRandomDouble(double min, double max)
    {
        std::uniform_real_distribution<double> dist(min, max);
        return dist(rng_);
    }
    
    long getCurrentTimestamp()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    
    // Member variables
    std::shared_ptr<gz::transport::Node> gz_node_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    
    SpawnRegion spawn_region_;
    std::vector<std::string> debris_models_;
    std::mt19937 rng_;
    
    int num_objects_to_spawn_;
    double spawn_interval_;
    int max_objects_;
    int objects_spawned_ = 0;
    int total_objects_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DebrisSpawner>();
    
    RCLCPP_INFO(node->get_logger(), "Starting debris spawner node...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
