#!/usr/bin/env python3
import csv, time, argparse, collections, subprocess
from pathlib import Path

def gz_service(*args):
    return subprocess.run(["gz","service",*args], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

def spawn(world, sdf_path, name, x, y, z):
    req = (
        f"sdf_filename: '{sdf_path}'\n"
        f"name: '{name}'\n"
        f"pose: {{\n"
        f"  position: {{\n"
        f"    x: {x}\n"
        f"    y: {y}\n"
        f"    z: {z}\n"
        f"  }}\n"
        f"  orientation: {{\n"
        f"    x: 0\n"
        f"    y: 0\n"
        f"    z: 0\n"
        f"    w: 1\n"
        f"  }}\n"
        f"}}\n"
    )
    return gz_service(
        "-s", f"/world/{world}/create",
        "--reqtype","gz.msgs.EntityFactory","--reptype","gz.msgs.Boolean",
        "--timeout","300","--req", req
    )

def set_pose(world, name, x, y, z):
    req = (
        f"name: '{name}'\n"
        f"position: {{\n"
        f"  x: {x}\n"
        f"  y: {y}\n"
        f"  z: {z}\n"
        f"}}\n"
        f"orientation: {{\n"
        f"  x: 0\n"
        f"  y: 0\n"
        f"  z: 0\n"
        f"  w: 1\n"
        f"}}\n"
    )
    return gz_service(
        "-s", f"/world/{world}/set_pose",
        "--reqtype","gz.msgs.Pose","--reptype","gz.msgs.Boolean",
        "--timeout","200","--req", req
    )

def main():
    ap = argparse.ArgumentParser("Spawn & move microplastic spheres from CSV in Gazebo Sim")
    ap.add_argument("--world", required=True, help="Gazebo world name")
    ap.add_argument("--sdf", required=True, help="microplastic_sphere.sdf path")
    ap.add_argument("--csv", required=True, help="particles.csv (t,idx,x,y)")
    ap.add_argument("--z", type=float, default=0.01, help="Z height")
    ap.add_argument("--realtime", action="store_true", help="sleep to match CSV dt")
    ap.add_argument("--fps", type=float, default=0.0, help="fixed playback fps (overrides realtime)")
    ap.add_argument("--prefix", default="mp_", help="model name prefix")
    args = ap.parse_args()

    # Load CSV grouped by time
    by_t = collections.defaultdict(list)
    ids = set()
    with open(args.csv) as f:
        r = csv.DictReader(f)
        for row in r:
            t = float(row["t"]); i = int(row["idx"])
            x = float(row["x"]); y = float(row["y"])
            by_t[t].append((i,x,y)); ids.add(i)

    times = sorted(by_t.keys())
    if not times:
        print("[ERR] CSV empty"); return

    # Spawn at first frame positions (or (0,0) if missing)
    t0 = times[0]
    first_xy = {i:(0.0,0.0) for i in ids}
    for (i,x,y) in by_t[t0]:
        first_xy[i] = (x,y)

    print(f"[INFO] Spawning {len(ids)} particles...")
    sdf_path = str(Path(args.sdf).resolve())
    for i in sorted(ids):
        name = f"{args.prefix}{i}"
        x0,y0 = first_xy[i]
        res = spawn(args.world, sdf_path, name, x0, y0, args.z)
        if res.returncode != 0:
            print(f"[ERROR] Failed to spawn {name}")
            print(f"  stdout: {res.stdout.strip()}")
            print(f"  stderr: {res.stderr.strip()}")
            print(f"  return code: {res.returncode}")
        else:
            print(f"[OK] Spawned {name}")

    # Playback
    print(f"[INFO] Playing {len(times)} frames...")
    last_t = times[0]
    fixed_dt = (1.0/args.fps) if args.fps > 0 else None
    for k,t in enumerate(times):
        if args.fps > 0:
            time.sleep(max(0.0, fixed_dt))
        elif args.realtime and k>0:
            time.sleep(max(0.0, t - last_t))
        for (i,x,y) in by_t[t]:
            name = f"{args.prefix}{i}"
            res = set_pose(args.world, name, x, y, args.z)
            if res.returncode != 0:
                print(f"[WARN] Failed to move {name}: {res.stderr.strip()}")
            elif k < 3:  # Show first few successful moves
                print(f"[MOVE] {name} -> ({x}, {y})")
        last_t = t
    print("[INFO] Done.")

if __name__ == "__main__":
    main()
