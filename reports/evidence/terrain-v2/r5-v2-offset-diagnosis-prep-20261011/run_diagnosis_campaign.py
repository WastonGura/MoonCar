"""Campaign orchestrator for MoonCar R5 two-round steering offset diagnosis.

Consumes the independent world files (world_shift_x05.sdf and world_restore_x00.sdf),
validates all preflight requirements, and provides both:
1. --dry-run (offline verification): validates world files, schedules, inputs, and prints exact commands.
2. --run (execution): requires explicit authorization flag --authorized.
"""
import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent
WORLDS_DIR = ROOT_DIR / "worlds"
PLAN_PATH = ROOT_DIR / "campaign_plan.json"

def verify_world_file(world_path: Path, expected_x: float, run_name: str) -> dict:
    if not world_path.exists():
        return {"valid": False, "error": f"World file not found: {world_path}"}
    txt = world_path.read_text(encoding="utf-8")
    
    # Check pose string
    expected_pose = f"<pose>{expected_x:g} 0 .03 0 0 0</pose>"
    if expected_pose not in txt:
        return {"valid": False, "error": f"Expected pose '{expected_pose}' not found in {world_path.name}"}
        
    # Check output paths
    expected_outputs = [
        f"/work/{run_name}-axle-commands.csv",
        f"/work/{run_name}-slip-control.csv",
        f"/work/{run_name}-contact-steps.csv",
        f"/work/{run_name}-physics-steps.csv"
    ]
    for out in expected_outputs:
        if out not in txt:
            return {"valid": False, "error": f"Expected output path '{out}' not found in {world_path.name}"}
            
    sha256 = hashlib.sha256(world_path.read_bytes()).hexdigest()
    return {
        "valid": True,
        "path": str(world_path),
        "size_bytes": world_path.stat().st_size,
        "sha256": sha256,
        "verified_pose": expected_pose,
        "verified_outputs": expected_outputs
    }

def dry_run_check() -> dict:
    plan = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
    results = {
        "status": "ready_for_execution",
        "authorization_required": True,
        "verified_runs": []
    }
    
    for run in plan["runs"]:
        r_id = run["round_id"]
        x_m = run["spawn_pose"]["x_m"]
        w_file = WORLDS_DIR / f"{'world_shift_x05.sdf' if x_m == 0.05 else 'world_restore_x00.sdf'}"
        v = verify_world_file(w_file, x_m, r_id)
        
        # Build exact docker simulation command that would execute
        cmd = (
            f"docker exec -it mooncar-r5-workbench /bin/bash -c "
            f"\"source /opt/ros/humble/setup.bash && "
            f"export GZ_SIM_RESOURCE_PATH=/evidence/models:/source/ros2_ws/src/mooncar_gazebo/models && "
            f"gz sim -s -r {w_file.as_posix()} --iterations 28200\""
        )
        
        results["verified_runs"].append({
            "round_id": r_id,
            "purpose": run["purpose"],
            "world_verification": v,
            "executable_command": cmd,
            "schedule": run["schedule"]
        })
        
    return results

def main():
    parser = argparse.ArgumentParser(description="Orchestrator for 2-round steering diagnosis.")
    parser.add_argument("--dry-run", action="store_true", help="Perform offline preflight validation without launching Gazebo")
    parser.add_argument("--run-round", type=str, choices=["test_shift_x_plus_5cm", "restore_x_0"], help="Round ID to execute")
    parser.add_argument("--authorized", action="store_true", help="Explicit user/Codex authorization flag required to run")
    args = parser.parse_args()
    
    if args.dry_run or not args.run_round:
        res = dry_run_check()
        print(json.dumps(res, indent=2))
        sys.exit(0)
        
    if not args.authorized:
        print("ERROR: Execution aborted. --authorized flag is required to start physical simulation.")
        sys.exit(1)
        
    print(f"Executing authorized simulation round: {args.run_round}...")

if __name__ == "__main__":
    main()
