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
        rows.append({"slip_command_pct": slip, "slip_measured_pct": round(measured_slip * 100, 3),
                     "wheel_angle_rad": round(angle, 5), "x_travel_m": round(x, 5),
                     "fx_mean_N": round(mean(fx), 5), "fx_sd_N": round(stdev(fx), 5),
                     "traction_coeff_nominal": round(mean(fx) / 24.5, 5),
                     "paper_FJS1_fig5_approx": TARGETS[slip], "samples_5_to_20s": len(fx)})
    with (HERE / "rigid_response.csv").open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(rows, ensure_ascii=False, indent=2))
    no_ground = mean(forces("slip_50_noground", 5, 10))
    known = mean(forces("slip_50_known_force", 5, 10))
    baseline = mean(forces("slip_50", 5, 10))
    print(f"controls: no_ground={no_ground:.4f} N, baseline={baseline:.4f} N, known_minus_baseline={known-baseline:.4f} N")
    assert abs(no_ground) < 0.1
    assert abs(known - baseline + 5) < 0.5
    for scope, prefix in (("ground", "ground_"), ("both_surfaces", "")):
        boundary = []
        for mu in (0.2, 0.6, 1.2):
            for slip in (0, 50):
                case = f"{prefix}mu{int(mu * 10):02}_slip_{slip:02}" if mu != 0.6 else f"slip_{slip:02}"
                values = forces(case)
                boundary.append({"sdf_mu_mu2": mu, "slip_pct": slip,
                                 "fx_mean_N": round(mean(values), 5),
                                 "traction_coeff_nominal": round(mean(values) / 24.5, 5)})
        with (HERE / f"{scope}_boundary.csv").open("w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=boundary[0].keys())
            writer.writeheader()
            writer.writerows(boundary)
        print(scope, json.dumps(boundary, ensure_ascii=False))
