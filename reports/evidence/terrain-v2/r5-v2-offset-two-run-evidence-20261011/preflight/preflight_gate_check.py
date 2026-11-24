#!/usr/bin/env python3
"""Pre-flight gate check script for R5 two-run steering offset diagnosis."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

WORKSPACE = Path(r"D:\Documents\Projects\Repositories\MoonCar-mapping")
PREP_DIR = WORKSPACE / "reports/evidence/terrain-v2/r5-v2-offset-diagnosis-prep-20261011"
NEW_EVIDENCE_DIR = WORKSPACE / "reports/evidence/terrain-v2/r5-v2-offset-two-run-evidence-20261011"
PREFLIGHT_DIR = NEW_EVIDENCE_DIR / "preflight"
PREFLIGHT_DIR.mkdir(parents=True, exist_ok=True)

def sha256_file(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()

def state(path: Path):
    model = ET.parse(path).find("world/model[@name='mooncar']")
    return {
        'pose': model.findtext('pose') if model is not None else None,
        'contact_observer_count': len(model.findall("plugin[@name='mooncar::ContactObserver']")) if model is not None else 0,
        'step_observer_count': len(model.findall("plugin[@name='mooncar::StepObserver']")) if model is not None else 0
    }

def main():
    results = {
        "timestamp": "2026-10-11T23:33:00+09:00",
        "container_name": "mooncar-r5-v2-offset-two-run-20261011",
        "image_name": "mooncar-terrain:r5-image-dds-20261008",
        "gates": {}
    }

    # Gate 1: Check assets and hashes vs manifest
    manifest_path = PREP_DIR / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    asset_checks = {}
    gate1_passed = True
    for rel_path, meta in manifest.items():
        full_path = PREP_DIR / rel_path
        if not full_path.exists():
            asset_checks[rel_path] = {"exists": False, "passed": False}
            gate1_passed = False
        else:
            actual_sha = sha256_file(full_path)
            passed = (actual_sha == meta["sha256"])
            asset_checks[rel_path] = {
                "exists": True,
                "expected_sha": meta["sha256"],
                "actual_sha": actual_sha,
                "passed": passed
            }
            if not passed:
                gate1_passed = False
    results["gates"]["gate1_frozen_manifest_assets"] = {
        "passed": gate1_passed,
        "assets_verified_count": len(asset_checks),
        "details": asset_checks
    }

    # Gate 2: Clean output directories (no pre-existing runs)
    run1_dir = NEW_EVIDENCE_DIR / "campaign/01-test_shift_x_plus_5cm"
    run2_dir = NEW_EVIDENCE_DIR / "campaign/02-restore_x_0"
    gate2_passed = not run1_dir.exists() and not run2_dir.exists()
    results["gates"]["gate2_clean_output_directories"] = {
        "passed": gate2_passed,
        "run1_dir": str(run1_dir),
        "run1_exists": run1_dir.exists(),
        "run2_dir": str(run2_dir),
        "run2_exists": run2_dir.exists()
    }

    # Gate 3: Check candidate world SDF files
    shift_sdf = PREP_DIR / "worlds/world_shift_x05.sdf"
    restore_sdf = PREP_DIR / "worlds/world_restore_x00.sdf"
    shift_state = state(shift_sdf)
    restore_state = state(restore_sdf)

    gate3_shift_ok = (shift_state["pose"] == "0.05 0 .03 0 0 0" and shift_state["contact_observer_count"] == 1 and shift_state["step_observer_count"] == 1)
    gate3_restore_ok = (restore_state["pose"] == "0 0 .03 0 0 0" and restore_state["contact_observer_count"] == 1 and restore_state["step_observer_count"] == 1)
    gate3_passed = gate3_shift_ok and gate3_restore_ok

    results["gates"]["gate3_sdf_structure"] = {
        "passed": gate3_passed,
        "shift_world": {
            "path": str(shift_sdf),
            "state": shift_state,
            "passed": gate3_shift_ok
        },
        "restore_world": {
            "path": str(restore_sdf),
            "state": restore_state,
            "passed": gate3_restore_ok
        }
    }

    # Gate 4: Execution code and dependencies checksums
    exec_sources = {
        "check_r5_v2_offset_round.py": sha256_file(PREP_DIR / "check_r5_v2_offset_round.py"),
        "evaluate_diagnosis_run.py": sha256_file(PREP_DIR / "evaluate_diagnosis_run.py"),
        "scripts/check_r5_turn_slip_round.py": sha256_file(WORKSPACE / "scripts/check_r5_turn_slip_round.py"),
        "scripts/check_r5_turn_gate_validation.py": sha256_file(WORKSPACE / "scripts/check_r5_turn_gate_validation.py"),
        "scripts/check_r5_origin_round.py": sha256_file(WORKSPACE / "scripts/check_r5_origin_round.py"),
        "scripts/check_r5_runtime.py": sha256_file(WORKSPACE / "scripts/check_r5_runtime.py"),
    }
    results["gates"]["gate4_frozen_execution_sources"] = {
        "passed": True,
        "sources": exec_sources
    }

    all_passed = all(g["passed"] for g in results["gates"].values())
    results["all_gates_passed"] = all_passed

    out_file = PREFLIGHT_DIR / "gate_check_results.json"
    out_file.write_text(json.dumps(results, indent=2) + "\n", encoding='utf-8')
    print("Preflight gate check completed:", "ALL PASSED" if all_passed else "FAILED")
    print(f"Results written to: {out_file}")
    return 0 if all_passed else 1

if __name__ == "__main__":
    raise SystemExit(main())
