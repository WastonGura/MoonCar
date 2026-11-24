"""Recompute prototype measurements, audit retained raw data, and draw the results."""

import csv
import hashlib
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pile import DIAMETER, DENSITY, GRAVITY, repose_angles


def digest(stream):
    result = hashlib.sha256()
    for block in iter(lambda: stream.read(1024*1024), b""):
        result.update(block)
    return result.hexdigest()


def main():
    root = Path(__file__).resolve().parent
    rows = []
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    for name, color in (("prototype32g_open_boundary", "#1673b8"), ("prototype32g_half_step", "#b46016")):
        folder = root/name
        summary = json.loads((folder/"summary.json").read_text())
        protocol = json.loads((folder/"protocol.json").read_text())
        data = np.load(folder/"final_particles.npz")
        xyz, velocity = data["xyz"], data["velocity"]
        count = len(xyz)
        assert count == summary["expected_particles"] == summary["inserted_particles"] == 2883
        assert np.isfinite(xyz).all() and np.isfinite(velocity).all()
        angles = repose_angles(xyz)
        assert len(angles) == 8
        mean = float(np.mean([row["angle_deg"] for row in angles]))
        assert abs(mean-summary["mean_angle_deg"]) < 1e-10
        stats = list(csv.DictReader((folder/"time_stats.csv").open()))
        assert all(float(row["kinetic_to_mgd"]) < 1e-5 for row in stats[-3:])
        mass = count*DENSITY*np.pi/6*DIAMETER**3
        energy = float(0.5*(mass/count)*np.sum(velocity**2))
        assert np.isclose(energy, float(stats[-1]["kinetic_J"]), rtol=1e-6, atol=1e-20)
        with zipfile.ZipFile(folder/"particles.zip") as archive:
            assert archive.testzip() is None
            with archive.open("particles.dump") as stream:
                raw_hash = digest(stream)
            if (folder/"particles.dump").exists():
                with (folder/"particles.dump").open("rb") as stream:
                    assert digest(stream) == raw_hash
        rows.append({"trial": name, "dt_s": protocol["dt_s"], "particles": count,
                     "mass_kg": mass, "sim_seconds": summary["sim_seconds"],
                     "wall_seconds": summary["wall_seconds"], "mean_angle_deg": mean,
                     "spatial_sector_sd_deg": float(np.std([a["angle_deg"] for a in angles], ddof=1)),
                     "min_profile_r_squared": min(a["r_squared"] for a in angles),
                     "translation_settled": True, "rotation_checked": False,
                     "raw_dump_sha256": raw_hash, "formal_calibration_pass": False})
        label = f"dt={protocol['dt_s']*1e6:g} us"
        if len(rows) == 1:
            axes[0, 0].scatter(xyz[:, 0]*1000, xyz[:, 2]*1000, s=3, color=color, alpha=0.4)
            axes[0, 0].set(xlabel="x (mm)", ylabel="z (mm)", title="32 g heap: x-z projection (all grains)")
            axes[0, 0].set_aspect("equal", adjustable="datalim")
            for angle in angles:
                profile = np.asarray(angle["outline"])*1000
                axes[0, 1].plot(profile[:, 0], profile[:, 1], alpha=0.7, label=f"Sector {angle['sector']+1}")
            axes[0, 1].set(xlabel="Radius (mm)", ylabel="Upper outline z (mm)", title="Eight spatial profiles, dt=8 us")
            axes[0, 1].legend(fontsize=8, ncol=2)
        axes[1, 0].semilogy([float(s["time_s"]) for s in stats],
                            [max(float(s["kinetic_to_mgd"]), 1e-15) for s in stats], color=color, label=label)
        axes[1, 1].scatter([a["sector"]+1 for a in angles], [a["angle_deg"] for a in angles], color=color, label=label)
        axes[1, 1].axhline(mean, color=color, alpha=0.6, linestyle=":")
    # Keep the failed boundary run in the audit; a retained failure is not a completed trial.
    failed = root/"prototype32g"
    assert "Lost atoms: original 1742 current 1741" in (root/"prototype32g_run.log").read_text()
    with zipfile.ZipFile(failed/"particles.zip") as archive:
        assert archive.testzip() is None
        with archive.open("particles.dump") as stream:
            failure_raw_hash = digest(stream)
        if (failed/"particles.dump").exists():
            with (failed/"particles.dump").open("rb") as stream:
                assert digest(stream) == failure_raw_hash
    report = {"completed_trials": rows, "half_step_angle_difference_deg": rows[1]["mean_angle_deg"]-rows[0]["mean_angle_deg"],
              "failed_trial": {"trial": "prototype32g", "reason": "one grain crossed fixed horizontal boundary",
                               "raw_dump_sha256": failure_raw_hash},
              "paper_geometry_matched": False, "parameter_fit_performed": False,
              "scope": "functional prototype; spatial sector SD is not repeated-run uncertainty"}
    (root/"audit.json").write_text(json.dumps(report, indent=2))
    axes[1, 0].axhline(1e-5, color="gray", linestyle="--", label="Translation stop threshold")
    axes[1, 0].set(xlabel="Simulation time (s)", ylabel="Translational KE / (M g d)", title="Translation only; rotation not measured")
    axes[1, 0].legend(fontsize=8)
    axes[1, 1].set(xlabel="Azimuth sector", ylabel="Angle (deg)", title="Step-size check; one seed, no parameter fit", xticks=range(1, 9))
    axes[1, 1].legend(fontsize=8)
    for axis in axes.flat:
        axis.grid(alpha=0.15)
    figure.suptitle("DEM measurement prototype | 2,883 spheres | 32 g | NOT paper replication", fontsize=14)
    figure.savefig(root/"prototype_measurements.png", dpi=170)
    plt.close(figure)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
