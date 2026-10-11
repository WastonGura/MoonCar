"""Campaign orchestrator for MoonCar R5 two-round steering offset diagnosis.

Consumes the independent world files (world_shift_x05.sdf and world_restore_x00.sdf),
validates all preflight requirements, and provides:
1. --dry-run (offline verification): validates world files, schedules, inputs, and prints exact commands.
2. --run-round / --run-all (execution): requires explicit authorization flag --authorized.
   Routes execution through check_r5_v2_offset_round.py -> check_r5_turn_gate_validation -> check_r5_turn_slip_round -> check_r5_runtime.
"""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent
WORLDS_DIR = ROOT_DIR / "worlds"
PLAN_PATH = ROOT_DIR / "campaign_plan.json"

CONTAINER_NAME = "mooncar-r5-v2-offset-diagnosis-20261011"
IMAGE_NAME = "mooncar-terrain:r5-image-dds-20261008"

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
        "docker_environment": {
            "container_name": CONTAINER_NAME,
            "image": IMAGE_NAME,
            "resources": {
                "cpus": 4,
                "memory": "6g",
                "shm_size": "512m"
            },
            "environment_variables": {
                "ROS_DISTRO": "jazzy",
                "ROS_DOMAIN_ID": "95",
                "MOONCAR_IMAGE_DDS_RUNTIME": "/opt/mooncar/image-dds/rmw-8.4.4",
                "MOONCAR_DART_PLUGIN_ROOT": "/opt/mooncar/physics/dart-7.8.0"
            },
            "docker_run_command": (
                f"docker run -d --name {CONTAINER_NAME} --cpus 4 --memory 6g --shm-size 512m "
                f"-v D:/Documents/Projects/Repositories/MoonCar-mapping:/source:ro "
                f"-v D:/Documents/Projects/Repositories/MoonCar-mapping/reports/evidence/terrain-v2/r5-v2-offset-diagnosis-prep-20261011:/evidence "
                f"-e ROS_DOMAIN_ID=95 -e ROS_DISTRO=jazzy {IMAGE_NAME} sleep infinity"
            )
        },
        "verified_runs": []
    }
    
    for run in plan["runs"]:
        r_id = run["round_id"]
        x_m = run["spawn_pose"]["x_m"]
        w_filename = 'world_shift_x05.sdf' if x_m == 0.05 else 'world_restore_x00.sdf'
        w_file = WORLDS_DIR / w_filename
        v = verify_world_file(w_file, x_m, r_id)
        
        # Real in-container execution command using Jazzy and ROS95
        cmd = (
            f"docker exec -i {CONTAINER_NAME} /bin/bash -lc "
            f"\"source /opt/ros/jazzy/setup.bash && source /work/install/setup.bash && "
            f"export PYTHONPATH=/source/scripts:\\$PYTHONPATH && "
            f"python3 /evidence/run_diagnosis_campaign.py --run-round {r_id} --authorized\""
        )
        
        results["verified_runs"].append({
            "round_id": r_id,
            "purpose": run["purpose"],
            "world_verification": v,
            "container_world_path": f"/evidence/worlds/{w_filename}",
            "executable_command": cmd,
            "schedule": run["schedule"]
        })
        
    return results

def execute_round(round_id: str, authorized: bool, runs_dir: Path = None, stub_executor = None) -> dict:
    """Execute a single diagnostic simulation round."""
    if not authorized:
        print(f"ERROR: Execution of round '{round_id}' aborted. --authorized flag is required.")
        sys.exit(1)

    plan = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
    run_cfg = next((r for r in plan["runs"] if r["round_id"] == round_id), None)
    if not run_cfg:
        raise ValueError(f"Unknown round_id: {round_id}")

    x_m = run_cfg["spawn_pose"]["x_m"]
    w_filename = 'world_shift_x05.sdf' if x_m == 0.05 else 'world_restore_x00.sdf'
    w_file = WORLDS_DIR / w_filename
    v = verify_world_file(w_file, x_m, round_id)
    if not v["valid"]:
        raise RuntimeError(f"World validation failed for {round_id}: {v['error']}")

    if runs_dir is None:
        runs_dir = ROOT_DIR / "runs"
    runs_dir.mkdir(parents=True, exist_ok=True)
    round_folder = runs_dir / round_id

    # Single-round entry point connecting to the ROS control chain
    round_script = ROOT_DIR / "check_r5_v2_offset_round.py"
    cmd = [
        sys.executable, str(round_script),
        "--output", str(runs_dir),
        "--name", round_id,
        "--world", str(w_file),
        "--angular", "0.2"
    ]

    print(f"Starting execution for round '{round_id}' consuming world: {w_file.name}...")

    if stub_executor:
        # Offline validation mode using stub executor
        res = stub_executor(cmd, round_folder, run_cfg)
    else:
        log_path = runs_dir / f"{round_id}.log"
        with log_path.open("w", encoding="utf-8") as log:
            child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
            start_wall = time.time()
            timed_out = False
            while child.poll() is None:
                if time.time() - start_wall > 600:
                    timed_out = True
                    child.terminate()
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                    break
                time.sleep(0.5)

        diag_path = round_folder / "diagnostic-result.json"
        if diag_path.exists():
            res = json.loads(diag_path.read_text(encoding="utf-8"))
        elif (round_folder / "result.json").exists():
            res = json.loads((round_folder / "result.json").read_text(encoding="utf-8"))
        else:
            res = {
                "round_id": round_id,
                "eligible": False,
                "error": "Execution produced no result.json",
                "timed_out": timed_out
            }

    # If eligible and physics steps exist, evaluate full causal evidence
    if res.get("eligible") and round_folder.exists() and (round_folder / "physics-steps.csv").exists():
        import evaluate_diagnosis_run
        eval_res = evaluate_diagnosis_run.evaluate_run(round_folder)
        eval_out = round_folder / "evaluation.json"
        eval_out.write_text(json.dumps(eval_res, indent=2) + "\n", encoding="utf-8")
        res["evaluation"] = eval_res

    print(f"Round '{round_id}' finished: eligible={res.get('eligible')}, "
          f"screen_exceeded={res.get('diagnostic_screen_exceeded')}, "
          f"max_disp_m={res.get('maximum_displacement_m')}")
    return res

def run_campaign(authorized: bool, runs_dir: Path = None, stub_executor = None) -> dict:
    """Execute the full 2-round diagnosis campaign adhering to strict stop rules."""
    if not authorized:
        print("ERROR: Execution aborted. --authorized flag is required to run simulation campaign.")
        sys.exit(1)

    plan = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
    summary = {
        "status": "running",
        "rounds": [],
        "campaign_qualified": False,
        "halt_reason": None
    }

    for run_cfg in plan["runs"]:
        r_id = run_cfg["round_id"]
        res = execute_round(r_id, authorized, runs_dir=runs_dir, stub_executor=stub_executor)
        summary["rounds"].append(res)

        # Strict stop rule: any genuine execution/data integrity failure halts campaign
        if not res.get("eligible"):
            summary["status"] = "halted"
            summary["halt_reason"] = f"Fatal qualification failure in round '{r_id}': {res.get('error') or res.get('turn_slip_error')}"
            print(f"STOP RULE TRIGGERED: {summary['halt_reason']}")
            break

    if len(summary["rounds"]) == len(plan["runs"]) and all(r.get("eligible") for r in summary["rounds"]):
        summary["status"] = "completed"
        summary["campaign_qualified"] = True

    return summary

def main():
    parser = argparse.ArgumentParser(description="Orchestrator for 2-round steering diagnosis.")
    parser.add_argument("--dry-run", action="store_true", help="Perform offline preflight validation without launching Gazebo")
    parser.add_argument("--run-round", type=str, choices=["test_shift_x_plus_5cm", "restore_x_0"], help="Round ID to execute")
    parser.add_argument("--run-all", action="store_true", help="Execute the complete 2-round diagnosis sequence")
    parser.add_argument("--authorized", action="store_true", help="Explicit user/Codex authorization flag required to run")
    parser.add_argument("--runs-dir", type=str, default=None, help="Output directory for simulation runs")
    args = parser.parse_args()
    
    runs_dir = Path(args.runs_dir) if args.runs_dir else None

    if args.dry_run or (not args.run_round and not args.run_all):
        res = dry_run_check()
        print(json.dumps(res, indent=2))
        sys.exit(0)
        
    if not args.authorized:
        print("ERROR: Execution aborted. --authorized flag is required to start physical simulation.")
        sys.exit(1)
        
    if args.run_round:
        res = execute_round(args.run_round, args.authorized, runs_dir=runs_dir)
        print(json.dumps(res, indent=2))
        sys.exit(0 if res.get("eligible") else 1)
        
    if args.run_all:
        res = run_campaign(args.authorized, runs_dir=runs_dir)
        print(json.dumps(res, indent=2))
        sys.exit(0 if res.get("campaign_qualified") else 1)

if __name__ == "__main__":
    main()
