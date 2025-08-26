# USV Planners Workspace

A ROS2 workspace containing packages for USV (Unmanned Surface Vehicle) navigation and control. Designed to work with the VRX (Virtual RobotX Challenge) simulation environment.

## Packages

### usv_planners
A path planning package for USV navigation featuring:
- **DWA Planner**: Dynamic Window Approach for high-speed local navigation
- **Real-time trajectory planning** with aggressive speed parameters (up to 50 m/s)
- **Enhanced visualization** with 100x larger trajectory markers
- **RViz integration** for interactive goal setting

[See detailed documentation →](usv_planners/README.md)

## Prerequisites

- **ROS2** (Humble/Iron/Rolling)
- **VRX Simulation** (external dependency - install from [VRX repository](https://github.com/osrf/vrx))
- **RViz2** for visualization

## Quick Start

### 1. Install VRX (External Dependency)
Follow VRX installation instructions at: https://github.com/osrf/vrx/wiki

### 2. Clone and Build This Workspace
```bash
git clone https://github.com/YOUR_USERNAME/usv-planners-workspace.git vrx_ws/src
cd vrx_ws
colcon build --packages-select usv_planners --merge-install
source install/setup.bash
