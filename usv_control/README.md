# USV Control Package

## Overview
The `usv_control` package provides teleoperation and velocity control capabilities for the WAM-V USV (Unmanned Surface Vehicle) in the VRX simulation environment.

## Features

### 1. Teleop Keyboard Controller (`usv_teleop_keyboard`)
- **Purpose**: Manual keyboard control of the WAM-V boat
- **Controls**:
  - `w/s`: Increase/decrease linear velocity (forward/backward)
  - `a/d`: Increase/decrease angular velocity (turn left/right)
  - `space`: Emergency stop (set all velocities to zero)
  - `q`: Quit the controller
- **Output**: Publishes `geometry_msgs/Twist` messages to `/wamv/cmd_vel`

### 2. Velocity Controller (`usv_velocity_controller`)
- **Purpose**: Converts `cmd_vel` commands to VRX thruster commands
- **Input**: Subscribes to `geometry_msgs/Twist` on `/wamv/cmd_vel`
- **Output**: Publishes thrust commands to:
  - `/wamv/thrusters/left/thrust` (std_msgs/Float64)
  - `/wamv/thrusters/right/thrust` (std_msgs/Float64)
- **Algorithm**: Differential drive control using linear and angular velocity commands

### 3. TF Broadcaster (`usv_tf_broadcaster`)
- **Purpose**: Connects the coordinate frames between VRX simulation and particle simulation
- **Input**: Subscribes to `/wamv/pose` from VRX simulation
- **Output**: Publishes TF transforms:
  - `world` → `map` (static identity transform)
  - `map` → `wamv/wamv/base_link` (dynamic transform from robot pose)
- **Function**: Enables visualization of particle simulation data relative to the robot's position

## Installation & Building

```bash
# Navigate to your ROS 2 workspace
cd /home/navid/vrx_ws

# Build the package
colcon build --packages-select usv_control

# Source the workspace
source install/setup.bash
```

## Launch Files

The package provides several launch files for different use cases:

- **`usv_bringup_with_tf.launch.py`** (Recommended): Complete system with Gazebo bridge, TF broadcaster, velocity controller, and teleop support
- **`usv_bringup.launch.py`**: Complete system without TF broadcaster (legacy)
- **`usv_tf_broadcaster.launch.py`**: TF broadcaster node only (for connecting particle simulation to robot frame)
- **`usv_control_complete.launch.py`**: Legacy launch file with velocity controller and teleop (no bridge)
- **`usv_velocity_controller.launch.py`**: Velocity controller node only
- **`usv_teleop_keyboard.launch.py`**: Teleop keyboard node only

## Usage

### Method 1: Launch Complete Control System with TF (Recommended)
```bash
# Launch the complete USV control system with Gazebo bridge and TF broadcaster
ros2 launch usv_control usv_bringup_with_tf.launch.py
```

### Method 2: Launch TF Broadcaster Only
```bash
# Launch only the TF broadcaster to connect map frame to robot frame
ros2 launch usv_control usv_tf_broadcaster.launch.py
```

### Method 3: Launch Complete Control System (Legacy)
```bash
# Launch the complete USV control system with Gazebo bridge (no TF)
ros2 launch usv_control usv_bringup.launch.py
```

Then send command on cmd_vel topic

## Parameters

### Teleop Keyboard Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_speed` | 2.0 | Maximum linear velocity (m/s) |
| `angular_speed` | 1.0 | Maximum angular velocity (rad/s) |
| `linear_step` | 0.1 | Linear velocity increment per keypress |
| `angular_step` | 0.1 | Angular velocity increment per keypress |

### Velocity Controller Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_thrust` | 250.0 | Maximum thrust per thruster (Newtons) |
| `boat_length` | 4.9 | Distance between thrusters (meters) |
| `thrust_deadband` | 0.1 | Minimum thrust to overcome friction |

### TF Broadcaster Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_frame` | "map" | Frame ID for particle simulation coordinate system |
| `robot_frame` | "wamv/wamv/base_link" | Robot base frame from VRX simulation |
| `world_frame` | "world" | World frame from VRX simulation |
| `publish_rate` | 50.0 | TF publishing frequency (Hz) |

## Topics

### Published Topics
- `/wamv/cmd_vel` (geometry_msgs/Twist) - Velocity commands from teleop
- `/wamv/thrusters/left/thrust` (std_msgs/Float64) - Left thruster command
- `/wamv/thrusters/right/thrust` (std_msgs/Float64) - Right thruster command

### Subscribed Topics
- `/wamv/cmd_vel` (geometry_msgs/Twist) - Velocity commands to convert

## Integration with VRX and Particle Simulation

This package is designed to work with both the VRX simulation environment and particle simulation:

1. **Prerequisites**: VRX simulation must be running
2. **WAM-V Model**: Compatible with standard VRX WAM-V configuration
3. **Thruster Interface**: Uses VRX thruster topic interface
4. **Coordinate System**: Follows VRX coordinate conventions
5. **Gazebo Bridge**: The launch files include ros_gz_bridge for seamless ROS 2 ↔ Gazebo communication
6. **TF Integration**: The TF broadcaster connects the `map` frame (used by particle simulation) to the robot's `wamv/wamv/base_link` frame

### TF Tree Structure
After launching with TF broadcaster, the coordinate frame tree will be:
```
world → map → wamv/wamv/base_link → [robot sensors and links]
```

This enables:
- Particle simulation data (in `map` frame) to be visualized relative to the robot
- Robot sensor data to be transformed to the particle simulation coordinate system
- Seamless integration between VRX simulation and particle-based environmental data

### Bridge Configuration
The Gazebo bridge automatically handles:
- `/wamv/cmd_vel` (ROS 2 ↔ Gazebo)
- `/wamv/thrusters/left/thrust` (ROS 2 → Gazebo)
- `/wamv/thrusters/right/thrust` (ROS 2 → Gazebo)

## Example Workflow

1. **Start VRX simulation**:
   ```bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   ```

2. **Launch USV control**:
   ```bash
   ros2 launch usv_control usv_bringup.launch.py
   ```

3. **Control the boat** using keyboard in the teleop terminal

## Troubleshooting

### Common Issues
- **No response to commands**: Check that VRX simulation is running and thruster topics exist
- **Keyboard input not working**: Ensure teleop node has terminal focus and xterm is available
- **Excessive thrust**: Reduce `max_thrust` parameter for gentler control

### Debugging Commands
```bash
# Check if topics are available
ros2 topic list | grep wamv

# Monitor cmd_vel commands
ros2 topic echo /wamv/cmd_vel

# Monitor thruster commands
ros2 topic echo /wamv/thrusters/left/thrust
ros2 topic echo /wamv/thrusters/right/thrust
```

## Dependencies
- ROS 2 (rclcpp, geometry_msgs, std_msgs)
- VRX simulation environment
- xterm (for keyboard teleop in separate terminal)
