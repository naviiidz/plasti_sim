# PlastiSim - Marine Debris Simulation

A Gazebo-based simulation system for modeling microplastic and marine debris trajectories in ocean environments.

## Overview

PlastiSim provides tools for spawning and controlling debris particles in Gazebo simulation environments, particularly designed for VRX (Virtual RobotX) ocean scenarios. The system supports both individual particle control and CSV-based trajectory following.

## Prerequisites

- ROS 2 (tested with Humble/Iron)
- Gazebo Garden
- VRX simulation environment
- Python 3.8+

## Quick Start

### 1. Spawning Individual Models

#### Basic Model Spawning

Use the `spawn_model.py` script to spawn individual debris models:

```bash
# Spawn a single microplastic sphere
python3 scripts/spawn_model.py --model microplastic_sphere --name "debris_001" --x -520.0 --y 200.0 --z 1.2

# Spawn a plastic bottle
python3 scripts/spawn_model.py --model plastic_bottle --name "bottle_001" --x -525.0 --y 205.0 --z 1.0

# List all available models
python3 scripts/spawn_model.py --list
```

#### Available Models

The `models/` directory contains several pre-built debris models:

- **microplastic_sphere/** - Small spherical microplastic particles
- **plastic_bottle/** - Plastic water bottles with realistic physics
- **plastic_container/** - Plastic containers and packaging
- **can_coke/** - Aluminum cans with textures
- **tuna_can/** - Metallic food cans
- **floating_object/** - Generic floating debris
- **marker_sphere/** - Simple colored spheres for visualization
- **red_marker/** - Red marker spheres for trajectory tracking

#### Model Structure

Each model directory contains:
- `model.sdf` - Gazebo SDF model definition
- `model.config` - Model metadata and configuration
- Texture files (if applicable)

### 2. Model Physics and Buoyancy

#### Configuring Buoyancy

Models include buoyancy plugins for realistic floating behavior:

```xml
<!-- Example from microplastic_sphere/model.sdf -->
<plugin filename="libPolyhedraBuoyancyDragPlugin.so" name="microplastic_buoyancy_drag">
    <wave_model>ocean_waves</wave_model>
    <fluid_density>1025.0</fluid_density>
    <fluid_level>0.0</fluid_level>
    <linear_drag>25.0</linear_drag>
    <angular_drag>2.0</angular_drag>
</plugin>
```

Key parameters:
- `fluid_density`: Ocean water density (kg/m³)
- `fluid_level`: Water surface level
- `linear_drag`: Linear drag coefficient
- `angular_drag`: Angular drag coefficient

#### Mass and Inertia Settings

Realistic physics require proper mass distribution:

```xml
<inertial>
    <mass>0.001</mass>  <!-- 1 gram for microplastics -->
    <inertia>
        <ixx>4e-7</ixx>
        <iyy>4e-7</iyy>
        <izz>4e-7</izz>
    </inertia>
</inertial>
```

### 3. Spawning Multiple Models

#### Batch Spawning

Spawn multiple models programmatically:

```python
# Example: Spawn multiple particles in a grid pattern
import subprocess

for i in range(5):
    for j in range(5):
        x = -520 + i * 2.0
        y = 200 + j * 2.0
        name = f"debris_{i}_{j}"
        
        subprocess.run([
            "python3", "scripts/spawn_model.py",
            "--model", "microplastic_sphere",
            "--name", name,
            "--x", str(x),
            "--y", str(y),
            "--z", "1.2"
        ])
```

#### Using Launch Files

Create launch files for complex scenarios:

```python
# launch/spawn_debris.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', 'scripts/spawn_model.py', 
                 '--model', 'microplastic_sphere',
                 '--name', 'debris_001',
                 '--x', '-520.0', '--y', '200.0', '--z', '1.2'],
            cwd='/home/navid/vrx_ws/src/plastiSim'
        ),
        # Add more spawn commands as needed
    ])
```

Launch with:
```bash
ros2 launch plastiSim spawn_debris.launch.py
```

### 4. Advanced Spawning Options

#### Custom Orientations

Spawn models with specific orientations:

```bash
python3 scripts/spawn_model.py \
    --model plastic_bottle \
    --name "bottle_tilted" \
    --x -530.0 --y 210.0 --z 1.0 \
    --roll 0.2 --pitch 0.1 --yaw 1.57
```

#### World Selection

Spawn in different Gazebo worlds:

```bash
python3 scripts/spawn_model.py \
    --model microplastic_sphere \
    --name "debris_001" \
    --x -520.0 --y 200.0 --z 1.2 \
    --world "sydney_regatta"
```

#### Verification and Debugging

Check if spawning was successful:

```bash
# List all models in the world
gz model --list

# Check specific model info
gz model --info --model "debris_001"

# Monitor model pose
gz model --pose --model "debris_001"
```

#### Troubleshooting

Common issues and solutions:

1. **Model not visible**: Check if Gazebo GUI is running and world is loaded
2. **Spawn failed**: Verify model SDF file exists and is valid
3. **Wrong position**: Check coordinate system - VRX uses ENU (East-North-Up)
4. **Model falls through water**: Adjust Z coordinate (water surface ≈ 0.0)

```bash
# Quick test - spawn a visible marker
python3 scripts/spawn_model.py --model marker_sphere --name "test_marker" --z 2.0

# Check available worlds
gz world --list

# Monitor Gazebo topics
gz topic --list
```

## Next: Movement and Trajectories

After spawning models, you can control their movement using:
- Position-based movement (`scripts/debris_trajectory.py`)
- Velocity-based movement
- CSV trajectory following
- Physics-based simulation

See the following sections for detailed movement control options.