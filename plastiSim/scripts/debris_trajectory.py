#!/usr/bin/env python3
"""
Spawn microplastics and move them following CSV trajectory data
"""
import csv
import time
import argparse
import subprocess
from collections import defaultdict
from pathlib import Path

def gz_service(*args):
    return subprocess.run(["gz", "service", *args], 
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

def spawn_microplastic_marker(world, name, x, y, z, marker_type="sphere", scale=0.5, color_r=1.0, color_g=0.0, color_b=0.0):
    """Spawn a visual marker using topic publication"""
    req = f"""header: {{
  stamp: {{
    sec: 0
    nsec: 0
  }}
  data: {{
    key: 'frame_id'
    value: '{world}'
  }}
}}
ns: 'debris_particles'
id: {hash(name) % 1000}
action: 0
type: 2
pose: {{
  position: {{
    x: {x}
    y: {y}
    z: {z}
  }}
  orientation: {{
    x: 0
    y: 0
    z: 0
    w: 1
  }}
}}
scale: {{
  x: {scale}
  y: {scale}
  z: {scale}
}}
material: {{
  ambient: {{
    r: {color_r}
    g: {color_g}
    b: {color_b}
    a: 1.0
  }}
  diffuse: {{
    r: {color_r}
    g: {color_g}
    b: {color_b}
    a: 1.0
  }}
}}
lifetime: {{
  sec: 0
  nsec: 0
}}"""
    return subprocess.run(
        ["gz", "topic", "-t", "/sensors/marker", "-m", "gz.msgs.Marker", "--req", req],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

def move_marker(world, name, x, y, z, scale=0.5, color_r=1.0, color_g=0.0, color_b=0.0):
    """Move a marker to new position using topic"""
    req = f"""header: {{
  stamp: {{
    sec: 0
    nsec: 0
  }}
  data: {{
    key: 'frame_id'
    value: '{world}'
  }}
}}
ns: 'debris_particles'
id: {hash(name) % 1000}
action: 0
type: 2
pose: {{
  position: {{
    x: {x}
    y: {y}
    z: {z}
  }}
  orientation: {{
    x: 0
    y: 0
    z: 0
    w: 1
  }}
}}
scale: {{
  x: {scale}
  y: {scale}
  z: {scale}
}}
material: {{
  ambient: {{
    r: {color_r}
    g: {color_g}
    b: {color_b}
    a: 1.0
  }}
  diffuse: {{
    r: {color_r}
    g: {color_g}
    b: {color_b}
    a: 1.0
  }}
}}
lifetime: {{
  sec: 0
  nsec: 0
}}"""
    return subprocess.run(
        ["gz", "topic", "-t", "/sensors/marker", "-m", "gz.msgs.Marker", "--req", req],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

def spawn_microplastic_sdf(world, sdf_path, name, x, y, z):
    """Fallback function to spawn SDF models"""
    req = f"""sdf_filename: '{sdf_path}'
name: '{name}'
pose: {{
  position: {{
    x: {x}
    y: {y}
    z: {z}
  }}
  orientation: {{
    x: 0
    y: 0
    z: 0
    w: 1
  }}
}}"""
    return gz_service(
        "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
        "--timeout", "300", "--req", req
    )

def move_microplastic(world, name, x, y, z):
    req = f"""name: '{name}'
position: {{
  x: {x}
  y: {y}
  z: {z}
}}
orientation: {{
  x: 0
  y: 0
  z: 0
  w: 1
}}"""
    return gz_service(
        "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "200", "--req", req
    )

def set_velocity(world, name, vx, vy, vz=0, angular_vx=0, angular_vy=0, angular_vz=0):
    """Set linear and angular velocity of an entity for continuous movement"""
    req = f"""name: '{name}'
linear_velocity: {{
  x: {vx}
  y: {vy}
  z: {vz}
}}
angular_velocity: {{
  x: {angular_vx}
  y: {angular_vy}
  z: {angular_vz}
}}"""
    return gz_service(
        "-s", f"/world/{world}/set_entity_velocity",
        "--reqtype", "gz.msgs.EntityVelocity", "--reptype", "gz.msgs.Boolean",
        "--timeout", "200", "--req", req
    )

def calculate_velocity(x1, y1, x2, y2, dt):
    """Calculate velocity components from position change over time"""
    vx = (x2 - x1) / dt if dt > 0 else 0
    vy = (y2 - y1) / dt if dt > 0 else 0
    return vx, vy

def main():
    parser = argparse.ArgumentParser(description="Spawn microplastics and follow CSV trajectories")
    parser.add_argument("--csv", required=True, help="CSV file with trajectory data (t,idx,x,y)")
    parser.add_argument("--world", default="sydney_regatta", help="Gazebo world name")
    parser.add_argument("--sdf", default="models/microplastic_sphere/model.sdf", help="SDF model file")
    parser.add_argument("--z", type=float, default=1.2, help="Z height for particles")
    parser.add_argument("--fps", type=float, default=2.0, help="Playback speed (frames per second)")
    parser.add_argument("--prefix", default="mp_", help="Name prefix for spawned particles")
    parser.add_argument("--single", type=int, default=None, help="Focus on single particle ID only")
    parser.add_argument("--verbose", action="store_true", help="Show detailed movement coordinates")
    parser.add_argument("--use-velocity", action="store_true", help="Use velocity for continuous movement instead of position jumps")
    parser.add_argument("--use-markers", action="store_true", default=True, help="Use simple sphere models for visualization (default)")
    parser.add_argument("--marker-size", type=float, default=0.5, help="Size of marker spheres")
    parser.add_argument("--marker-color", nargs=3, type=float, default=[1.0, 0.2, 0.2], help="RGB color for markers (0-1)")
    parser.add_argument("--marker-model", default="models/marker_sphere/model.sdf", help="SDF model file for markers")
    
    args = parser.parse_args()
    
    # 1. Read CSV file and count unique indices
    trajectory_data = defaultdict(list)
    unique_indices = set()
    
    with open(args.csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row['t'])
            idx = int(row['idx'])
            x = float(row['x'])
            y = float(row['y'])
            
            # Filter for single particle if specified
            if args.single is not None and idx != args.single:
                continue
                
            trajectory_data[t].append((idx, x, y))
            unique_indices.add(idx)
    
    if args.single is not None:
        print(f"Focusing on single particle ID: {args.single}")
    
    print(f"Found {len(unique_indices)} unique microplastic particles")
    print(f"Trajectory data spans {len(trajectory_data)} time steps")
    
    # 2. Spawn microplastics equal to unique number
    sdf_path = str(Path(args.sdf).resolve())
    times = sorted(trajectory_data.keys())
    first_time = times[0]
    
    # Get initial positions for each particle
    initial_positions = {}
    for idx, x, y in trajectory_data[first_time]:
        initial_positions[idx] = (x, y)
    
    # Fill missing initial positions with (0, 0)
    for idx in unique_indices:
        if idx not in initial_positions:
            initial_positions[idx] = (0.0, 0.0)
    
    print("Spawning particle markers...")
    for idx in sorted(unique_indices):
        name = f"{args.prefix}{idx}"
        x0, y0 = initial_positions[idx]
        
        if args.use_markers:
            # Use simple sphere models as markers
            marker_sdf_path = str(Path(args.marker_model).resolve())
            result = spawn_microplastic_sdf(args.world, marker_sdf_path, name, x0, y0, args.z)
            if result.returncode == 0:
                print(f"✓ Spawned marker {name} at ({x0:.1f}, {y0:.1f})")
            else:
                print(f"✗ Failed to spawn marker {name}: {result.stderr.strip()}")
        else:
            # Use original microplastic models
            result = spawn_microplastic_sdf(args.world, sdf_path, name, x0, y0, args.z)
            if result.returncode == 0:
                print(f"✓ Spawned {name} at ({x0:.1f}, {y0:.1f})")
            else:
                print(f"✗ Failed to spawn {name}: {result.stderr.strip()}")
    
    # 3. Read sequence and move microplastics gradually with time
    print(f"Starting trajectory playback at {args.fps} FPS...")
    print(f"Movement mode: {'Velocity-based (continuous)' if args.use_velocity else 'Position-based (interpolated)'}")
    dt = 1.0 / args.fps
    
    if args.use_velocity:
        # Velocity-based continuous movement
        for i in range(len(times) - 1):
            current_time = times[i]
            next_time = times[i + 1]
            time_diff = next_time - current_time
            
            # Get positions for current and next time
            current_positions = {}
            next_positions = {}
            
            for idx, x, y in trajectory_data[current_time]:
                current_positions[idx] = (x, y)
            
            for idx, x, y in trajectory_data[next_time]:
                next_positions[idx] = (x, y)
            
            print(f"Time segment: {current_time:.2f}s -> {next_time:.2f}s (duration: {time_diff:.2f}s)")
            
            # Calculate and set velocities for each particle
            for idx in unique_indices:
                name = f"{args.prefix}{idx}"
                
                if idx in current_positions and idx in next_positions:
                    x1, y1 = current_positions[idx]
                    x2, y2 = next_positions[idx]
                    
                    # Calculate velocity needed to reach next position
                    vx, vy = calculate_velocity(x1, y1, x2, y2, time_diff)
                    
                    if args.verbose:
                        distance = ((x2-x1)**2 + (y2-y1)**2)**0.5
                        speed = distance / time_diff if time_diff > 0 else 0
                        print(f"  {name}: velocity ({vx:.3f}, {vy:.3f}) m/s, speed: {speed:.3f} m/s")
                    
                    # Set velocity for continuous movement
                    set_velocity(args.world, name, vx, vy, 0)
                else:
                    # Stop particle if no trajectory data
                    if args.verbose:
                        print(f"  {name}: stopping (no trajectory data)")
                    set_velocity(args.world, name, 0, 0, 0)
            
            # Wait for the time segment to complete
            time.sleep(time_diff)
        
        # Stop all particles at the end
        print("Stopping all particles...")
        for idx in unique_indices:
            name = f"{args.prefix}{idx}"
            set_velocity(args.world, name, 0, 0, 0)
            if args.verbose:
                print(f"  {name}: stopped")
    
    else:
        # Continuous position-based movement with high-frequency updates
        print("Using continuous position-based movement with high-frequency updates")
        
        # Calculate total trajectory time
        total_time = times[-1] - times[0]
        continuous_fps = 20.0  # High frequency for smooth continuous movement
        update_dt = 1.0 / continuous_fps
        total_steps = int(total_time * continuous_fps)
        
        print(f"Continuous movement: {continuous_fps} Hz updates over {total_time:.1f} seconds ({total_steps} steps)")
        
        # Build simple linear interpolators for each particle
        particle_trajectories = {}
        for idx in unique_indices:
            # Collect time points and positions for this particle
            particle_data = []
            
            for t in times:
                for p_idx, x, y in trajectory_data[t]:
                    if p_idx == idx:
                        particle_data.append((t, x, y))
                        break
            
            if len(particle_data) >= 2:
                particle_trajectories[idx] = particle_data
        
        # Continuous movement loop
        start_time = times[0]
        for step in range(total_steps + 1):
            current_time = start_time + step * update_dt
            
            # Progress reporting (every 2 seconds)
            if step % int(continuous_fps * 2) == 0:
                progress = (step / total_steps) * 100 if total_steps > 0 else 100
                print(f"Time: {current_time:.2f}s ({progress:.0f}%)")
            
            # Move each particle smoothly
            for idx in unique_indices:
                name = f"{args.prefix}{idx}"
                
                if idx in particle_trajectories:
                    # Simple linear interpolation between trajectory points
                    trajectory = particle_trajectories[idx]
                    
                    # Find the two points to interpolate between
                    x, y = None, None
                    for i in range(len(trajectory) - 1):
                        t1, x1, y1 = trajectory[i]
                        t2, x2, y2 = trajectory[i + 1]
                        
                        if t1 <= current_time <= t2:
                            # Linear interpolation
                            if t2 - t1 > 0:
                                t_norm = (current_time - t1) / (t2 - t1)
                                x = x1 + t_norm * (x2 - x1)
                                y = y1 + t_norm * (y2 - y1)
                            else:
                                x, y = x1, y1
                            break
                    
                    # Use first or last point if outside range
                    if x is None:
                        if current_time <= trajectory[0][0]:
                            _, x, y = trajectory[0]
                        else:
                            _, x, y = trajectory[-1]
                    
                    if x is not None and y is not None:
                        # Smooth continuous movement
                        move_microplastic(args.world, name, x, y, args.z)
                        
                        if args.verbose and step % int(continuous_fps) == 0:  # Print every second
                            print(f"  {name}: ({x:.2f}, {y:.2f})")
            
            # High-frequency sleep for smooth movement
            time.sleep(update_dt)
    
    # Handle the final time step
    if times:
        final_time = times[-1]
        print(f"Time: {final_time:.2f}s (final)")
        for idx, x, y in trajectory_data[final_time]:
            name = f"{args.prefix}{idx}"
            move_microplastic(args.world, name, x, y, args.z)
    
    print("Trajectory playback complete!")

if __name__ == "__main__":
    main()
