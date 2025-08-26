# Particle Sim

A ROS2 package for visualizing particle movement from CSV data in RViz with 3D mesh environments, particle detection services, and action-based particle sampling.

## Summary

This package provides:
- **Particle Simulation**: Reads particle position data from CSV files and creates animated visualizations in RViz
- **Microparticle Sensor Service**: A service-based particle detection system that samples particles within a specified radius for a given duration
- **Particle Sampling Action Server**: An action-based particle sampling system for long-running sampling operations with feedback

## Features

### Particle Simulation
- Particle positions as colored spheres
- Movement trails
- 3D environment meshes (Sydney Regatta shore)
- Different material types (PE/PP Pellet, EPS Bead, Microbead PE/PP, Cenosphere, Bubble Foam)

### Particle Sensor Service & Action Server
- Service-based particle sampling for quick operations
- Action-based particle sampling for long-running operations with progress feedback
- Configurable detection radius
- Vectorized processing for efficient detection
- Returns detailed results including particle positions, materials, and detection timestamps
- Thread-safe sampling with proper locking

## Quick Start

### 1. Build the package
```bash
colcon build --packages-select particle_sim
source install/setup.bash
```

### 2. Run Particle Simulation + Sensor Service + Action Server
```bash
# Launch particle simulation
ros2 launch particle_sim particle_sim.launch.py

# Launch sensor service for testing
ros2 launch particle_sim sensor_test.launch.py

# Or run components separately
ros2 run particle_sim particle_sim_node.py --ros-args -p csv_file_path:=$(pwd)/src/particle_sim/sydney_regatta.csv
ros2 run particle_sim microparticle_sensor_node.py
ros2 run particle_sim particle_sampling_action_server.py
```

### 3. Test the Sensor Service & Actions
```bash
# Test service - sample particles for 5 seconds with default detection radius
ros2 service call /sample_particles particle_sim/srv/SampleParticles "{sampling_duration: 5.0}"

# Test service with custom detection radius
ros2 service call /sample_particles particle_sim/srv/SampleParticles "{sampling_duration: 10.0, detection_radius: 2.0}"

# Test action server - sample particles with progress feedback
ros2 action send_goal /sample_particles particle_sim/action/SampleParticles "{sampling_duration: 15.0, detection_radius: 3.0}"
```

## Service & Action Interfaces

**Service:** `~/sample_particles` (type: `particle_sim/SampleParticles`)
- **Request**: `sampling_duration` (float64), `detection_radius` (float64, optional)
- **Response**: `success` (bool), `message` (string), `total_detections` (int32), `unique_particles` (int32), `actual_duration` (float64), `particles` (DetectedParticle[])

**Action:** `~/sample_particles` (type: `particle_sim/SampleParticles`)
- **Goal**: `sampling_duration` (float64), `detection_radius` (float64, optional)  
- **Result**: `success` (bool), `message` (string), `total_detections` (int32), `unique_particles` (int32), `actual_duration` (float64), `particles` (DetectedParticle[])
- **Feedback**: `elapsed_time` (float64), `detections_so_far` (int32)

See [SERVICE_README.md](SERVICE_README.md) and [ACTION_README.md](ACTION_README.md) for detailed documentation.

## CSV Format

Supports two formats:

**Standard format:**
```csv
particle_id,x,y,z,timestamp
0,1.0,2.0,0.0,0.0
```

**Sydney Regatta format:**
```csv
idx,x,y,z,t
0,1.0,2.0,0.0,0.0
```

## Key Parameters

### Particle Simulation
- `csv_file_path`: Path to particle data CSV
- `particle_scale`: Particle size (default: 0.2)
- `publish_rate`: Animation speed in Hz (default: 10.0)
- `trail_length`: Number of trail points (default: 50)
- `auto_play`: Automatically start playback (default: true)
- `frame_id`: Reference frame for visualization (default: 'map')

### Sensor Service & Action Server
- `robot_frame`: TF frame of the robot (default: 'wamv_simple')
- `detection_radius`: Detection radius in meters (default: 5.0)
- `frame_id`: Reference frame for particle detection (default: 'map')

## Topics

- `/particle_markers`: Current particle positions (MarkerArray)
- `/particle_trails`: Movement trails (MarkerArray)
- `/shore_mesh`: Environment mesh (Marker)

## Services & Actions

- `/sample_particles`: Particle sampling service (SampleParticles)
- `/sample_particles`: Particle sampling action (SampleParticles)

## Dependencies

- ROS2 (Jazzy)
- rclpy, visualization_msgs, geometry_msgs
- NumPy for vectorized operations
- tf2_ros for robot position tracking
- RViz2 for visualization
