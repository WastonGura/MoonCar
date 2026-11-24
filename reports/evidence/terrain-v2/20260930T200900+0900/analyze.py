"""Audit the frozen wheel-bench raw streams without fitting soil parameters."""

import csv
import json
from pathlib import Path
from statistics import mean, stdev


HERE = Path(__file__).resolve().parent
# Approximate red experimental markers read by eye from Hurrell et al., Fig. 5b, PDF p. 10.
# They are visual context, not released raw measurements or optimizer targets.
TARGETS = {0: 0.30, 10: 0.33, 20: 0.35, 30: 0.36, 40: 0.39, 50: 0.42}


def forces(case, start=5, end=20):
    rows = [json.loads(line) for line in (HERE / f"{case}_x_force.jsonl").read_text(encoding="utf-8").splitlines()]
    selected = []
    for row in rows:
        stamp = row["header"]["stamp"]
        seconds = int(stamp.get("sec", 0)) + int(stamp.get("nsec", 0)) * 1e-9
        if start <= seconds <= end:
            selected.append(row["force"].get("x", 0))
    assert len(selected) >= (end - start) * 90, (case, len(selected))
    return selected


def positions(case):
    data = json.loads((HERE / f"{case}_joint_state.json").read_text(encoding="utf-8"))
    return {joint["name"]: joint.get("axis1", {}).get("position", 0) for joint in data["joint"]}


def ground_load(case, start=5, end=20):
    rows = [json.loads(line) for line in (HERE / f"{case}_ground_force.jsonl").read_text().splitlines()]
    rail = [json.loads(line) for line in (HERE / f"{case}_x_force.jsonl").read_text().splitlines()]
    assert len(rows) == len(rail)
    loads, contact_x, errors = [], [], []
    for row, other in zip(rows, rail):
        stamp = row["header"]["stamp"]
        assert stamp == other["header"]["stamp"]
        seconds = int(stamp.get("sec", 0)) + int(stamp.get("nsec", 0)) * 1e-9
        if start <= seconds <= end:
            loads.append(-row["force"].get("z", 0) - 10 * 9.81)
            contact_x.append(-row["force"].get("x", 0))
            errors.append(row["force"].get("x", 0) + other["force"].get("x", 0))
    assert len(loads) >= (end-start) * 90
    # Instantaneous rail/contact differences include carriage acceleration.
    assert abs(mean(errors)) < 0.1, (case, mean(errors))
    return mean(loads), mean(contact_x), mean(errors), max(abs(value) for value in errors)


if __name__ == "__main__":
    rows = []
    for slip in TARGETS:
        case = f"slip_{slip:02}"
        fx = forces(case)
        joint = positions(case)
        x, angle = joint["translate_x"], joint["rotate_wheel"]
        measured_slip = 1 - x / (0.1 * angle)
        assert abs(measured_slip - slip / 100) < 0.01, (case, measured_slip)
        assert abs(angle - 4.0) < 0.01, (case, angle)
        fz, contact_x, balance_error, balance_max = ground_load(case)
        assert abs(fz - 24.5) < 0.5, (case, fz)
        rows.append({"slip_command_pct": slip, "slip_measured_pct": round(measured_slip * 100, 3),
                     "wheel_angle_rad": round(angle, 5), "x_travel_m": round(x, 5),
                     "fx_mean_N": round(contact_x, 5), "rail_fx_sd_N": round(stdev(fx), 5),
                     "traction_coeff_nominal": round(mean(fx) / 24.5, 5),
                     "fz_mean_N": round(fz, 5), "force_balance_mean_N": round(balance_error, 5),
                     "force_balance_max_N": round(balance_max, 5),
                     "traction_coeff_measured": round(contact_x / fz, 5),
                     "paper_FJS1_fig5_approx": TARGETS[slip], "samples_5_to_20s": len(fx)})
    with (HERE / "rigid_response.csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(rows, ensure_ascii=False, indent=2))
    phases = []
    for slip in TARGETS:
        case = f"slip_{slip:02}"
        phases.append({"slip_pct": slip, **{f"fx_{start}_{start+5}s_N": round(mean(forces(case, start, start + 5)), 5)
                                               for start in (5, 10, 15)}})
    with (HERE / "phase_audit.csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=phases[0].keys())
        writer.writeheader()
        writer.writerows(phases)
    print("phases", json.dumps(phases, ensure_ascii=False))
    no_ground = mean(forces("slip_50_noground", 5, 10))
    known = mean(forces("slip_50_known_force", 5, 10))
    baseline = mean(forces("slip_50", 5, 10))
    print(f"controls: no_ground={no_ground:.4f} N, baseline={baseline:.4f} N, known_minus_baseline={known-baseline:.4f} N")
    assert abs(no_ground) < 0.1
    assert abs(known - baseline + 5) < 0.5
    assert abs(ground_load("slip_50_noground", 5, 10)[0]) < 0.1
    boundary = []
    for mu in (0.2, 0.6, 1.2):
        for slip in (0, 50):
            case = f"ground_mu{int(mu * 10):02}_slip_{slip:02}" if mu != 0.6 else f"slip_{slip:02}"
            values = forces(case)
            fz, contact_x, balance_error, balance_max = ground_load(case)
            assert abs(fz - 24.5) < 0.5, (case, fz)
            boundary.append({"sdf_mu_mu2": mu, "slip_pct": slip,
                             "fx_mean_N": round(contact_x, 5),
                             "traction_coeff_nominal": round(mean(values) / 24.5, 5),
                             "fz_mean_N": round(fz, 5), "force_balance_mean_N": round(balance_error, 5),
                             "force_balance_max_N": round(balance_max, 5),
                             "traction_coeff_measured": round(contact_x / fz, 5),
                             "fx_5_10s_N": round(mean(forces(case, 5, 10)), 5),
                             "fx_10_15s_N": round(mean(forces(case, 10, 15)), 5),
                             "fx_15_20s_N": round(mean(forces(case, 15, 20)), 5)})
    with (HERE / "ground_boundary.csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=boundary[0].keys())
        writer.writeheader()
        writer.writerows(boundary)
    print(json.dumps(boundary, ensure_ascii=False))
    windows = []
    for start, end in ((5, 20), (20, 35), (35, 50), (50, 64), (5, 36.416), (32, 63.416)):
        fz, fx, imbalance, maximum = ground_load("slip_50_long", start, end)
        windows.append({"start_s": start, "end_s": end, "fx_mean_N": fx,
                        "fz_mean_N": fz, "traction_coeff_measured": fx/fz})
    with (HERE / "long_trial_windows.csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=windows[0].keys())
        writer.writeheader()
        writer.writerows(windows)
    cases = {f"slip_{value:02}": 20 for value in TARGETS}
    cases.update({f"ground_mu{mu:02}_slip_{slip:02}": 20 for mu in (2, 12) for slip in (0, 50)})
    cases.update({"slip_50_noground": 10, "slip_50_known_force": 10, "slip_50_long": 64})
    counts = {}
    for case, seconds in cases.items():
        counts[case] = {}
        for label in ("x_force", "z_force", "ground_force"):
            data = [json.loads(line) for line in (HERE/f"{case}_{label}.jsonl").read_text().splitlines()]
            assert len(data) == seconds*100 + 1, (case, label, len(data))
            assert int(data[-1]["header"]["stamp"].get("sec", 0)) == seconds
            stamps = [int(row["header"]["stamp"].get("sec", 0))*1_000_000_000
                      + int(row["header"]["stamp"].get("nsec", 0)) for row in data]
            assert 0 <= stamps[0] < 10_000_000, (case, label, stamps[0])
            assert all(stamp == index*10_000_000 for index, stamp in enumerate(stamps[1:], 1)), (case, label)
            counts[case][label] = len(data)
    (HERE / "audit.json").write_text(json.dumps({"completed_trials": len(cases),
        "raw_force_counts": counts, "control_no_ground_fx_N": no_ground,
        "control_known_force_change_N": known-baseline,
        "timestamps_complete_100_Hz": True,
        "parameter_fit_performed": False}, indent=2), encoding="utf-8")
    print("audit", len(cases), "trials; raw force streams complete")
