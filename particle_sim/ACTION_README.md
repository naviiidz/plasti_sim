# Particle Sampling Action

This ROS 2 action server provides robust particle sampling functionality using the same detection logic as the microparticle sensor node. The action server supports multiple sequential calls and provides real-time feedback during sampling operations.

## Files Created

1. **`scripts/particle_sampling_action_server.py`** - Action server that samples particles over time
2. **`scripts/particle_sampling_client.py`** - Example action client for testing
3. **`scripts/test_sequential_sampling.py`** - Test script for sequential sampling calls
4. **`scripts/test_multiple_calls.py`** - Test script for concurrent sampling calls  
5. **`launch/particle_sampling_action.launch.py`** - Launch file for the action server

## Action Interface

The action uses the existing `SampleParticles.action` interface:

### Goal
- `sampling_duration` (float64): Duration in seconds to sample particles
- `detection_radius` (float64): Detection radius around robot (uses default if 0.0)

### Result
- `success` (bool): Whether sampling completed successfully
- `message` (string): Status message
- `total_detections` (int32): Total number of particles detected
- `unique_particles` (int32): Number of unique particles detected  
- `actual_duration` (float64): Actual sampling duration
- `particles` (DetectedParticle[]): List of all detected particles with material types

### Feedback
- `elapsed_time` (float64): Time elapsed since sampling started
- `current_detections` (int32): Current number of detections so far
- `progress` (float64): Progress percentage (0.0 to 1.0)

## Usage

### Launch the action server:
```bash
ros2 launch particle_sim particle_sampling_action.launch.py
```

### Call the action from command line:
```bash
ros2 action send_goal /sample_particles particle_sim/action/SampleParticles "{sampling_duration: 5.0, detection_radius: 10.0}"
```

### Use the test client:
```bash
ros2 run particle_sim particle_sampling_client.py
```

### Run test scripts:
```bash
# Sequential sampling tests
python3 src/particle_sim/scripts/test_sequential_sampling.py

# Multiple calls test
python3 src/particle_sim/scripts/test_multiple_calls.py
```

## Key Features

- **Robust Multi-Call Support**: Handles multiple sequential action calls without timeout issues
- **Same Detection Logic**: Uses identical vectorized particle detection as the sensor node
- **Material Classification**: Returns particle counts by material type (PE/PP Pellet, EPS Bead, etc.)
- **Real-time Feedback**: Provides progress updates during sampling
- **Configurable Parameters**: Detection radius and sampling duration
- **Performance Optimized**: Uses NumPy vectorized operations and correct TF frame handling

## Example Output

The action returns results like:
```
Total detections: 25
Unique particles: 25  
Actual duration: 5.00s
Particles by material type:
  - PE/PP Pellet: 8
  - EPS Bead: 6
  - Microbead PE/PP: 4
  - Cenosphere: 4
  - Bubble Foam: 3
```

## Testing Results

The action server supports:
- Multiple sequential action calls work correctly
- Robust error handling with proper timeout management
- Consistent performance without degradation after multiple calls
- Real-time feedback during sampling operations
