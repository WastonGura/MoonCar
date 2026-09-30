"""Check the frozen contrast and summarize measured motion."""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
RADIUS_M = 0.076


def normalized_world(case):
    root = ET.parse(HERE / f"{case}.sdf").getroot()
    ground = [m for m in root.findall("./world/model") if m.get("name", "").endswith("_ground")]
    assert len(ground) == 4
    for model in ground:
        for tag in ("ode/mu", "ode/mu2", "bullet/friction", "bullet/friction2"):
            model.find(f"./link/collision/surface/friction/{tag}").text = "fixed"
    return ET.tostring(root)


def summarize(case):
    rows = [json.loads(line) for line in (HERE / f"{case}_samples.jsonl").read_text().splitlines()]
    assert len(rows) == 11
    assert [row["iterations"] for row in rows] == list(range(1000, 11001, 1000))
    first, last = rows[0], rows[-1]
    assert set(first["wheel_rad"]) == set(last["wheel_rad"]) and len(first["wheel_rad"]) == 4
    wheel_m = RADIUS_M * sum(last["wheel_rad"][key] - first["wheel_rad"][key]
                             for key in first["wheel_rad"]) / 4
    dx, dy = last["x"] - first["x"], last["y"] - first["y"]
    peak = max(rows, key=lambda row: row["x"])
    return {"case": case, "duration_s": 20.0, "delta_x_m": dx, "delta_y_m": dy,
            "net_xy_m": math.hypot(dx, dy), "wheel_nominal_m": wheel_m,
            "forward_progress_per_wheel_distance": dx / wheel_m,
            "sampled_peak_x_m": peak["x"], "sampled_peak_time_s": peak["sim_seconds"],
            "sampled_backslide_m": peak["x"] - last["x"],
            "start_xyz_m": [first[key] for key in ("x", "y", "z")],
            "end_xyz_m": [last[key] for key in ("x", "y", "z")]}


def main():
    cases = ("scale_025", "scale_050", "scale_075", "scale_100")
    assert len({normalized_world(case) for case in cases}) == 1, "Worlds differ beyond ground friction"
    results = [summarize(case) for case in cases]
    assert max(row["wheel_nominal_m"] for row in results) - min(row["wheel_nominal_m"] for row in results) < 0.01
    path = HERE / "summary.json"
    path.write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
