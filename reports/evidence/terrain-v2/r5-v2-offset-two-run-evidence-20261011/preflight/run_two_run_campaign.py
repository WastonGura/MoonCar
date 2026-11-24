#!/usr/bin/env python3
"""Dedicated two-round campaign runner for R5 steering offset diagnosis.

Executes exactly two rounds in sequence adhering to GEMINI_R5_OFFSET_TWO_RUN_TASK_20261011.md.
"""
import json
import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(r"D:\Documents\Projects\Repositories\MoonCar-mapping")
EVIDENCE_DIR = WORKSPACE / "reports/evidence/terrain-v2/r5-v2-offset-two-run-evidence-20261011"
CAMPAIGN_DIR = EVIDENCE_DIR / "campaign"
PREFLIGHT_DIR = EVIDENCE_DIR / "preflight"
CONTAINER = "mooncar-r5-v2-offset-two-run-20261011"

ROUNDS = [
    {
        "round_num": 1,
        "round_id": "01-test_shift_x_plus_5cm",
        "world_file": "/evidence/worlds/world_shift_x05.sdf",
        "expected_x": 0.05,
        "angular": 0.2,
        "description": "Shift vehicle +5cm along X axis to alter mesh contact phase"
    },
    {
        "round_num": 2,
        "round_id": "02-restore_x_0",
        "world_file": "/evidence/worlds/world_restore_x00.sdf",
        "expected_x": 0.0,
        "angular": 0.2,
        "description": "Restore vehicle to origin (X=0) to verify reproduction"
    }
]

def run_round_in_container(r_info: dict) -> dict:
    r_id = r_info["round_id"]
    w_file = r_info["world_file"]
    ang = r_info["angular"]
    
    cmd_str = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source /work/install/setup.bash && "
        f"export PYTHONPATH=/source/scripts:/evidence:$PYTHONPATH && "
        f"python3 /evidence/check_r5_v2_offset_round.py "
        f"--output /campaign --name {r_id} --world {w_file} --angular {ang}"
    )
    
    exec_cmd = ["docker", "exec", "-i", CONTAINER, "bash", "-c", cmd_str]
    log_file = PREFLIGHT_DIR / f"{r_id}.log"
    
    print(f"\n========================================================")
    print(f"STARTING ROUND {r_info['round_num']}: {r_id}")
    print(f"World: {w_file} (Spawn X: {r_info['expected_x']}m)")
    print(f"Command: {cmd_str}")
    print(f"========================================================")
    
    wall_start = time.time()
    timed_out = False
    
    with log_file.open("w", encoding="utf-8") as lf:
        proc = subprocess.Popen(exec_cmd, stdout=lf, stderr=subprocess.STDOUT)
        while proc.poll() is None:
            elapsed = time.time() - wall_start
            if elapsed > 600: # 10 minutes limit
                timed_out = True
                print(f"[TIMEOUT] Round {r_id} exceeded 10 minutes wall time limit! Terminating...")
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                break
            time.sleep(1.0)
            
    wall_duration = time.time() - wall_start
    exit_code = proc.returncode
    
    print(f"Round {r_id} finished in {wall_duration:.2f}s with exit code {exit_code} (timed_out={timed_out})")
    
    # Check outputs on host
    round_host_dir = CAMPAIGN_DIR / r_id
    diag_file = round_host_dir / "diagnostic-result.json"
    result_file = round_host_dir / "result.json"
    
    diag_data = {}
    if diag_file.exists():
        diag_data = json.loads(diag_file.read_text(encoding="utf-8"))
    elif result_file.exists():
        diag_data = json.loads(result_file.read_text(encoding="utf-8"))
        
    # Evaluate contact causality and events
    eval_data = {}
    if (round_host_dir / "physics-steps.csv").exists():
        eval_cmd_str = (
            f"source /opt/ros/jazzy/setup.bash && "
            f"source /work/install/setup.bash && "
            f"export PYTHONPATH=/source/scripts:/evidence:$PYTHONPATH && "
            f"python3 /evidence/evaluate_diagnosis_run.py "
            f"--run-dir /campaign/{r_id} --output /campaign/{r_id}/evaluation.json"
        )
        print(f"Evaluating causal evidence for {r_id}...")
        eval_proc = subprocess.run(
            ["docker", "exec", "-i", CONTAINER, "bash", "-c", eval_cmd_str],
            capture_output=True, text=True
        )
        print("Evaluator returncode:", eval_proc.returncode)
        if eval_proc.stdout:
            print("Evaluator stdout:", eval_proc.stdout[:300])
        if eval_proc.stderr:
            print("Evaluator stderr:", eval_proc.stderr[:300])
        
        eval_json_file = round_host_dir / "evaluation.json"
        if eval_json_file.exists():
            eval_data = json.loads(eval_json_file.read_text(encoding="utf-8"))
            
    summary = {
        "round_num": r_info["round_num"],
        "round_id": r_id,
        "world_file": w_file,
        "expected_x": r_info["expected_x"],
        "wall_start": wall_start,
        "wall_duration_s": wall_duration,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "diagnostic_result": diag_data,
        "evaluation": eval_data
    }
    
    (round_host_dir / "run_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary

def main():
    campaign_summary = {
        "start_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "container": CONTAINER,
        "rounds": [],
        "status": "in_progress",
        "halt_reason": None
    }
    
    for r_info in ROUNDS:
        res = run_round_in_container(r_info)
        campaign_summary["rounds"].append(res)
        
        diag = res.get("diagnostic_result", {})
        eval_res = res.get("evaluation", {})
        
        # Stop rules:
        # 1. Timed out
        if res.get("timed_out"):
            campaign_summary["status"] = "halted"
            campaign_summary["halt_reason"] = f"Round {res['round_id']} timed out (> 600s)"
            print(f"STOP RULE TRIGGERED: {campaign_summary['halt_reason']}")
            break
            
        # 2. Check eligible
        if not diag.get("eligible"):
            campaign_summary["status"] = "halted"
            campaign_summary["halt_reason"] = f"Round {res['round_id']} failed eligibility qualification: {diag.get('turn_slip_error') or diag.get('checks')}"
            print(f"STOP RULE TRIGGERED: {campaign_summary['halt_reason']}")
            break
            
        # 3. Check contact evaluation qualified
        if eval_res and not eval_res.get("qualified"):
            campaign_summary["status"] = "halted"
            campaign_summary["halt_reason"] = f"Round {res['round_id']} failed contact source evaluation: {eval_res.get('qualification_error')}"
            print(f"STOP RULE TRIGGERED: {campaign_summary['halt_reason']}")
            break
            
        print(f"Round {res['round_id']} completed successfully! Proceeding...")

    if len(campaign_summary["rounds"]) == 2 and campaign_summary["status"] == "in_progress":
        campaign_summary["status"] = "completed"

    campaign_summary["end_time"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    (CAMPAIGN_DIR / "campaign_summary.json").write_text(json.dumps(campaign_summary, indent=2) + "\n", encoding="utf-8")
    print("\nCampaign execution finished with status:", campaign_summary["status"])
    return 0 if campaign_summary["status"] == "completed" else 1

if __name__ == "__main__":
    raise SystemExit(main())
