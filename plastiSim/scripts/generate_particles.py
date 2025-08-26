#!/usr/bin/env python3
"""
Generate realistic particle trajectories for marine debris simulation
"""
import csv
import math
import random
import argparse

def generate_debris_trajectories(num_particles=20, duration=30, timestep=0.5):
    """
    Generate realistic marine debris trajectories with:
    - Ocean current drift
    - Wind effects 
    - Wave motion
    - Brownian motion (turbulence)
    """
    particles = []
    
    # Simulation parameters
    current_speed = 0.2  # m/s eastward current
    wind_speed = 0.1     # m/s northward wind effect
    wave_amplitude = 0.3 # Wave motion amplitude
    wave_period = 4.0    # Wave period in seconds
    turbulence = 0.05    # Random motion strength
    
    # Initial spawn area (near WAM-V)
    spawn_x_range = (-530, -515)
    spawn_y_range = (170, 210)
    
    for particle_id in range(num_particles):
        # Random initial position
        start_x = random.uniform(*spawn_x_range)
        start_y = random.uniform(*spawn_y_range)
        
        # Generate trajectory over time
        x, y = start_x, start_y
        
        for t_step in range(int(duration / timestep) + 1):
            t = t_step * timestep
            
            # Ocean current (steady eastward drift)
            x += current_speed * timestep
            
            # Wind effect (northward drift)
            y += wind_speed * timestep
            
            # Wave motion (sinusoidal)
            wave_x = wave_amplitude * math.sin(2 * math.pi * t / wave_period)
            wave_y = wave_amplitude * math.cos(2 * math.pi * t / wave_period) * 0.5
            
            # Add turbulence (random walk)
            turb_x = random.gauss(0, turbulence)
            turb_y = random.gauss(0, turbulence)
            
            # Final position
            final_x = x + wave_x + turb_x
            final_y = y + wave_y + turb_y
            
            particles.append({
                't': round(t, 2),
                'idx': particle_id + 1,
                'x': round(final_x, 2),
                'y': round(final_y, 2)
            })
    
    return particles

def generate_spiral_pattern(num_particles=10, duration=20, timestep=0.5):
    """Generate particles in a spiral pattern (like a vortex)"""
    particles = []
    center_x, center_y = -520, 190
    
    for particle_id in range(num_particles):
        # Each particle starts at different angle
        start_angle = (particle_id / num_particles) * 2 * math.pi
        radius_start = 5 + particle_id * 2
        
        for t_step in range(int(duration / timestep) + 1):
            t = t_step * timestep
            
            # Spiral motion
            angle = start_angle + t * 0.2  # Rotate slowly
            radius = radius_start + t * 0.1  # Expand slowly
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            particles.append({
                't': round(t, 2),
                'idx': particle_id + 1,
                'x': round(x, 2),
                'y': round(y, 2)
            })
    
    return particles

def main():
    parser = argparse.ArgumentParser(description="Generate particle trajectory CSV files")
    parser.add_argument("--type", choices=["debris", "spiral"], default="debris",
                       help="Type of trajectory pattern")
    parser.add_argument("--particles", type=int, default=20, help="Number of particles")
    parser.add_argument("--duration", type=float, default=30, help="Simulation duration (seconds)")
    parser.add_argument("--timestep", type=float, default=0.5, help="Time step (seconds)")
    parser.add_argument("--output", default="generated_particles.csv", help="Output CSV file")
    
    args = parser.parse_args()
    
    print(f"[INFO] Generating {args.particles} particles for {args.duration}s...")
    
    if args.type == "debris":
        particles = generate_debris_trajectories(args.particles, args.duration, args.timestep)
    else:  # spiral
        particles = generate_spiral_pattern(args.particles, args.duration, args.timestep)
    
    # Write to CSV
    with open(args.output, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['t', 'idx', 'x', 'y'])
        writer.writeheader()
        writer.writerows(particles)
    
    print(f"[INFO] Generated {len(particles)} data points")
    print(f"[INFO] Saved to {args.output}")
    print(f"[INFO] Time range: {min(p['t'] for p in particles)} - {max(p['t'] for p in particles)}s")

if __name__ == "__main__":
    main()
