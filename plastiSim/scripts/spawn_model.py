#!/usr/bin/env python3
"""
Simple model spawner for Gazebo using gz service calls
"""
import subprocess
import argparse
from pathlib import Path

def gz_service(*args):
    """Execute gz service command and return result"""
    return subprocess.run(["gz", "service", *args], 
                         stdout=subprocess.PIPE, 
                         stderr=subprocess.PIPE, 
                         text=True)

def spawn_model(world, sdf_path, name, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    """
    Spawn a model in Gazebo world
    
    Args:
        world: Gazebo world name
        sdf_path: Path to SDF model file
        name: Unique name for spawned model
        x, y, z: Position coordinates
        roll, pitch, yaw: Orientation in radians
    """
    # Convert Euler angles to quaternion (simplified for small angles)
    # For more accurate conversion, use scipy.spatial.transform.Rotation
    w = 1.0  # Simplified - assumes small angles
    
    req = f"""sdf_filename: '{sdf_path}'
name: '{name}'
pose: {{
  position: {{
    x: {x}
    y: {y}
    z: {z}
  }}
  orientation: {{
    x: {roll}
    y: {pitch}
    z: {yaw}
    w: {w}
  }}
}}"""
    
    print(f"[INFO] Spawning {name} at ({x}, {y}, {z})")
    print(f"[DEBUG] Using SDF: {sdf_path}")
    
    result = gz_service(
        "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "300",
        "--req", req
    )
    
    print(f"[DEBUG] Return code: {result.returncode}")
    print(f"[DEBUG] stdout: '{result.stdout.strip()}'")
    print(f"[DEBUG] stderr: '{result.stderr.strip()}'")
    
    if result.returncode == 0:
        print(f"[SUCCESS] Model '{name}' spawned successfully")
        return True
    else:
        print(f"[ERROR] Failed to spawn model '{name}'")
        return False

def list_available_models(models_dir):
    """List available model directories"""
    models_path = Path(models_dir)
    if not models_path.exists():
        print(f"[ERROR] Models directory not found: {models_dir}")
        return []
    
    models = []
    for item in models_path.iterdir():
        if item.is_dir():
            sdf_file = item / "model.sdf"
            if sdf_file.exists():
                models.append(item.name)
    
    return sorted(models)

def main():
    parser = argparse.ArgumentParser(description="Spawn models in Gazebo simulation")
    parser.add_argument("--world", default="sydney_regatta", help="Gazebo world name")
    parser.add_argument("--model", help="Model name (e.g., plastic_bottle)")
    parser.add_argument("--name", help="Unique instance name (defaults to model name + timestamp)")
    parser.add_argument("--x", type=float, default=0, help="X position")
    parser.add_argument("--y", type=float, default=0, help="Y position") 
    parser.add_argument("--z", type=float, default=1, help="Z position")
    parser.add_argument("--roll", type=float, default=0, help="Roll angle (radians)")
    parser.add_argument("--pitch", type=float, default=0, help="Pitch angle (radians)")
    parser.add_argument("--yaw", type=float, default=0, help="Yaw angle (radians)")
    parser.add_argument("--models-dir", default="/home/navid/vrx_ws/src/plastiSim/models", 
                       help="Path to models directory")
    parser.add_argument("--list", action="store_true", help="List available models")
    
    args = parser.parse_args()
    
    # List available models if requested
    if args.list:
        print("[INFO] Available models:")
        models = list_available_models(args.models_dir)
        for model in models:
            print(f"  - {model}")
        return
    
    # Check if model argument is required
    if not args.model:
        print("[ERROR] --model argument is required")
        parser.print_help()
        return
    
    # Validate model exists
    model_path = Path(args.models_dir) / args.model / "model.sdf"
    if not model_path.exists():
        print(f"[ERROR] Model SDF not found: {model_path}")
        print("Use --list to see available models")
        return
    
    # Generate unique name if not provided
    if not args.name:
        import time
        timestamp = int(time.time())
        args.name = f"{args.model}_{timestamp}"
    
    # Spawn the model
    success = spawn_model(
        world=args.world,
        sdf_path=str(model_path.resolve()),
        name=args.name,
        x=args.x,
        y=args.y, 
        z=args.z,
        roll=args.roll,
        pitch=args.pitch,
        yaw=args.yaw
    )
    
    if success:
        print(f"[INFO] Successfully spawned '{args.name}' in world '{args.world}'")
    else:
        print("[ERROR] Spawning failed")

if __name__ == "__main__":
    main()
