# Particle Sensor Service

The microparticle sensor has been converted to a ROS 2 service that samples particles for a specified duration and returns the results.

## Service Interface

**Service Name:** `~/sample_particles`  
**Service Type:** `particle_sim/SampleParticles`

### Request
- `sampling_duration` (float64): Duration in seconds to sample particles
- `detection_radius` (float64): Detection radius around robot (optional, uses default if 0.0)

### Response
- `success` (bool): Whether sampling completed successfully
- `message` (string): Status message
- `total_detections` (int32): Total number of particles detected
- `unique_particles` (int32): Number of unique particles detected  
- `actual_duration` (float64): Actual sampling duration
- `particles` (DetectedParticle[]): Array of all detected particles

### DetectedParticle Message
- `particle_id` (int32): Unique particle identifier
- `material_type` (string): Type of material (PE/PP Pellet, EPS Bead, etc.)
- `distance` (float64): Distance from robot when detected
- `particle_x`, `particle_y`, `particle_z` (float64): Particle position
- `robot_x`, `robot_y`, `robot_z` (float64): Robot position at detection time
- `detection_timestamp` (float64): Unix timestamp of detection

## Usage

### 1. Build the Package
```bash
cd /path/to/your/workspace
colcon build --packages-select particle_sim
source install/setup.bash
```

### 2. Launch the Service
```bash
# Launch particle simulation with sensor service
ros2 launch particle_sim particle_sim_with_service.launch.py

# Or launch just the sensor service (requires particles from another source)
ros2 launch particle_sim particle_sensor_service.launch.py
```

### 3. Call the Service

#### Using the Client Script
```bash
# Sample for 5 seconds with default detection radius
ros2 run particle_sim particle_sampler_client.py 5.0

# Sample for 10 seconds with 2.0m detection radius
ros2 run particle_sim particle_sampler_client.py 10.0 2.0
```

#### Using ROS 2 CLI
```bash
# Sample for 3 seconds with default detection radius
ros2 service call /sample_particles particle_sim/srv/SampleParticles "{sampling_duration: 3.0, detection_radius: 0.0}"

# Sample for 5 seconds with 1.5m detection radius
ros2 service call /sample_particles particle_sim/srv/SampleParticles "{sampling_duration: 5.0, detection_radius: 1.5}"
```

### 4. Python Client Example
```python
import rclpy
from rclpy.node import Node
from particle_sim.srv import SampleParticles

class MyClient(Node):
    def __init__(self):
        super().__init__('my_client')
        self.client = self.create_client(SampleParticles, 'sample_particles')
        
    def sample_particles(self, duration, radius=0.0):
        request = SampleParticles.Request()
        request.sampling_duration = duration
        request.detection_radius = radius
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
```

## Parameters

The sensor service accepts the following parameters:

- `robot_frame` (string, default: 'wamv_simple'): TF frame of the robot
- `default_detection_radius` (float, default: 1.0): Default detection radius in meters
- `frame_id` (string, default: 'map'): Reference frame for particle detection

## Notes

- The service blocks during sampling, so only one sampling operation can run at a time
- The sensor requires valid TF transforms between the reference frame and robot frame
- All particles within the detection radius are recorded during the sampling period
- Duplicate detections of the same particle are allowed (useful for tracking particle density over time)
- The service automatically handles vectorized processing for efficient particle detection
