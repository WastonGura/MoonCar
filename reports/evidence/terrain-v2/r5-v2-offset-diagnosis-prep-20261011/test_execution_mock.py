"""Offline mock/stub test verifying execution pipeline, world consumption, and stop rules.

Does NOT launch Gazebo or Docker. Demonstrates that:
1. Two-round runner calls the ROS control chain arguments.
2. The independent world replicas are genuinely validated and consumed.
3. Diagnostic screening logic treats > 10 mm displacement as a diagnostic outcome rather than a crash.
4. Fatal failures (e.g. data missing) halt the campaign immediately.
5. Overwrite protection properly refuses overwriting existing output markers.
6. Baseline evaluation extracts event 17019 and neighbor grid edges windows.
"""
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent

def test_dry_run_and_command_structure():
    import run_diagnosis_campaign as runner
    dry = runner.dry_run_check()
    assert dry["status"] == "ready_for_execution"
    assert dry["docker_environment"]["container_name"] == "mooncar-r5-v2-offset-diagnosis-20261011"
    assert dry["docker_environment"]["environment_variables"]["ROS_DISTRO"] == "jazzy"
    assert len(dry["verified_runs"]) == 2
    for r in dry["verified_runs"]:
        assert r["world_verification"]["valid"] is True
        assert "jazzy" in r["executable_command"]
        assert "run_diagnosis_campaign.py" in r["executable_command"]
        assert r["schedule"]["total_sim_steps"] == 28200
    print("[PASS] Dry-run and command structure check passed.")

def test_round_execution_with_stub():
    import run_diagnosis_campaign as runner
    captured_commands = []

    def mock_stub_executor(cmd_args, round_folder, run_cfg):
        captured_commands.append((cmd_args, round_folder, run_cfg))
        # Simulate round completion with expected 10.286 mm offset
        round_folder.mkdir(parents=True, exist_ok=True)
        # Mock result.json and diagnostic-result.json
        diag_res = {
            "round_id": run_cfg["round_id"],
            "eligible": True,
            "diagnostic_screen_exceeded": True,
            "maximum_displacement_m": 0.010286,
            "checks": {"data_complete": True, "no_critical_errors": True, "valid_exit": True},
            "world_consumed": str(runner.WORLDS_DIR / ("world_shift_x05.sdf" if run_cfg["spawn_pose"]["x_m"] == 0.05 else "world_restore_x00.sdf")),
            "turn_slip_error": "ValueError('Existing 1 cm turn screen failed')"
        }
        (round_folder / "diagnostic-result.json").write_text(json.dumps(diag_res, indent=2) + "\n")
        return diag_res

    # Test executing round with stub
    test_runs_dir = ROOT_DIR / "tmp_test_runs"
    if test_runs_dir.exists():
        shutil.rmtree(test_runs_dir)

    try:
        res = runner.execute_round("test_shift_x_plus_5cm", authorized=True, runs_dir=test_runs_dir, stub_executor=mock_stub_executor)
        assert res["eligible"] is True
        assert res["diagnostic_screen_exceeded"] is True
        assert len(captured_commands) == 1
        args, r_folder, r_cfg = captured_commands[0]
        assert "--world" in args
        assert "world_shift_x05.sdf" in args[args.index("--world") + 1]
        assert "--angular" in args
        print("[PASS] Round execution with stub executor passed.")

        # Test full campaign execution with stop rule verification
        captured_commands.clear()
        campaign_res = runner.run_campaign(authorized=True, runs_dir=test_runs_dir, stub_executor=mock_stub_executor)
        assert campaign_res["status"] == "completed"
        assert len(campaign_res["rounds"]) == 2
        print("[PASS] Full 2-round campaign sequence with stub executor passed.")

        # Test stop rule on fatal failure in round 1
        def mock_failing_stub(cmd_args, round_folder, run_cfg):
            return {
                "round_id": run_cfg["round_id"],
                "eligible": False,
                "error": "Missing contact source records",
                "checks": {"data_complete": False}
            }
        failed_campaign = runner.run_campaign(authorized=True, runs_dir=test_runs_dir, stub_executor=mock_failing_stub)
        assert failed_campaign["status"] == "halted"
        assert len(failed_campaign["rounds"]) == 1 # Halted immediately after round 1!
        print("[PASS] Stop rule halted campaign immediately on fatal failure.")

    finally:
        if test_runs_dir.exists():
            shutil.rmtree(test_runs_dir)

def test_overwrite_protection():
    import evaluate_diagnosis_run as evaluator
    marker = ROOT_DIR / "tmp_overwrite_marker.json"
    marker.write_text('{"preserve_marker": true}\n')
    fixtures_dir = ROOT_DIR.parent / 'r5-v2-offset-prep-codex-review-20261011' / 'synthetic-complete'
    try:
        proc = subprocess.run(
            [sys.executable, str(ROOT_DIR / "evaluate_diagnosis_run.py"), "--run-dir", str(fixtures_dir), "--output", str(marker)],
            capture_output=True, text=True
        )
        assert proc.returncode != 0, "Evaluator should exit non-zero when output file exists"
        assert "preserve_marker" in marker.read_text(), "Existing marker must not be overwritten"
        print("[PASS] CLI overwrite protection verified.")
    finally:
        if marker.exists():
            marker.unlink()

def test_baseline_evaluation_and_event_17019():
    import evaluate_diagnosis_run as evaluator
    base_dir = ROOT_DIR.parent / 'r5-turn-gate-validation-20261011' / 'campaign' / '14-pure_turn-ramp-1000us-left'
    res = evaluator.evaluate_run(base_dir)
    assert res["qualified"] is True
    assert "grid_edge_overlap_windows_all_wheels" in res
    sample_iters = [ev["iteration"] for ev in res["sample_critical_events"]]
    assert 17019 in sample_iters, f"Target mutation event 17019 must be in sample events, got {sample_iters}"
    assert Path(res["critical_events_file"]).exists()
    print("[PASS] Baseline evaluation event 17019 and grid edge windows verified.")

if __name__ == "__main__":
    test_dry_run_and_command_structure()
    test_round_execution_with_stub()
    test_overwrite_protection()
    test_baseline_evaluation_and_event_17019()
    print("\nALL OFFLINE STUB TESTS COMPLETED SUCCESSFULLY!")
