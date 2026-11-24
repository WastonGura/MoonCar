"""Audit completed slots and plot measured geometry; pending slots stay pending."""

import csv
import hashlib
import io
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from pile import DENSITY, GRAVITY, repose_angles


def digest(stream):
    result = hashlib.sha256()
    for block in iter(lambda: stream.read(1024*1024), b""):
        result.update(block)
    return result.hexdigest()


def read_dump(archive, diameter, expected):
    frames = []
    last = None
    previous = 0
    with archive.open("particles.dump") as raw:
        stream = io.TextIOWrapper(raw)
        while stream.readline():
            step = int(stream.readline())
            assert stream.readline().strip() == "ITEM: NUMBER OF ATOMS"
            count = int(stream.readline())
            assert previous <= count <= expected, (previous, count, expected)
            previous = count
            assert stream.readline().startswith("ITEM: BOX BOUNDS")
            for _ in range(3):
                stream.readline()
            columns = stream.readline().split()[2:]
            data = np.asarray([list(map(float, stream.readline().split())) for _ in range(count)])
            if count:
                assert np.isfinite(data).all()
                assert len(np.unique(data[:, columns.index("id")])) == count
                assert np.allclose(data[:, columns.index("radius")], diameter/2, atol=1e-12)
                last = (columns, data)
            frames.append({"step": step, "particles": count})
    assert last is not None and frames[-1]["particles"] == expected
    return frames, last


def audit_trial(folder, case):
    summary = json.loads((folder/"summary.json").read_text())
    protocol = json.loads((folder/"protocol.json").read_text())
    diameter = protocol["diameter_m"]
    mass = DENSITY*np.pi/6*diameter**3
    expected = round(case["mass_g"]/1000/mass)
    assert protocol["target_particles"] == summary["inserted_particles"] == summary["expected_particles"] == expected
    assert abs(protocol["dt_s"]-case["dt_s"]) < 1e-12
    assert abs(diameter-case["diameter_mm"]/1000) < 1e-12
    assert protocol["seed"] == 123457 and protocol["target_used_for_fitting"] is False
    data = np.load(folder/"final_particles.npz")
    xyz, velocity, omega = data["xyz"], data["velocity"], data["omega"]
    assert xyz.shape == velocity.shape == omega.shape == (expected, 3)
    assert np.isfinite(xyz).all() and np.isfinite(velocity).all() and np.isfinite(omega).all()
    translation = float(0.5*mass*np.sum(velocity**2))
    rotation = float(0.5*0.4*mass*(diameter/2)**2*np.sum(omega**2))
    stats = list(csv.DictReader((folder/"time_stats.csv").open()))
    assert np.isclose(translation, float(stats[-1]["kinetic_J"]), rtol=1e-6, atol=1e-20)
    assert np.isclose(rotation, float(stats[-1]["rotation_J"]), rtol=1e-6, atol=1e-20)
    if summary["translation_settled_and_discharged"]:
        assert all(int(s["particles_above_outlet"]) == 0 and float(s["kinetic_to_mgd"]) < 1e-5 for s in stats[-3:])
    heap = xyz[xyz[:, 2]+diameter/2 < protocol["outlet_height_m"]]
    assert len(heap) == summary["particles_in_heap"]
    angles = repose_angles(heap, radius=diameter/2)
    mean = float(np.mean([a["angle_deg"] for a in angles])) if angles else None
    assert mean is None and summary["mean_angle_deg"] is None or abs(mean-summary["mean_angle_deg"]) < 1e-10
    with zipfile.ZipFile(folder/"particles.zip") as archive:
        assert archive.testzip() is None
        frames, (columns, final_raw) = read_dump(archive, diameter, expected)
        with archive.open("particles.dump") as stream:
            raw_hash = digest(stream)
    if (folder/"particles.dump").exists():
        with (folder/"particles.dump").open("rb") as stream:
            assert digest(stream) == raw_hash
    if "ids" in data:
        order = np.argsort(data["ids"])
        assert np.array_equal(final_raw[:, columns.index("id")], data["ids"][order])
        for names, values in ((('x', 'y', 'z'), xyz), (('vx', 'vy', 'vz'), velocity),
                              (('omegax', 'omegay', 'omegaz'), omega)):
            raw_values = final_raw[:, [columns.index(n) for n in names]]
            assert np.allclose(raw_values, values[order], atol=5e-7, rtol=1e-5), names
    loaded_outside = None
    if (folder/"loaded_hopper.npz").exists():
        loaded = np.load(folder/"loaded_hopper.npz")["xyz"]
        assert len(loaded) == expected
        bottom, top = protocol["outlet_height_m"], protocol["funnel_top_m"]
        wall_r = 0.008+(loaded[:, 2]-bottom)*0.052/0.08
        loaded_outside = int(np.sum((np.hypot(loaded[:, 0], loaded[:, 1]) > wall_r+1e-5)
                                   | (loaded[:, 2] < bottom-1e-5) | (loaded[:, 2] > top+diameter/2)))
    spin_square = float(np.sum(omega**2))
    return {"trial": case["name"], "mass_g": mass*expected*1000, "particles": expected,
            "diameter_mm": diameter*1000, "dt_us": protocol["dt_s"]*1e6,
            "outlet_height_mm": protocol["outlet_height_m"]*1000,
            "release_time_s": summary["release_time_s"], "sim_seconds": summary["sim_seconds"],
            "wall_seconds": summary["wall_seconds"], "translation_settled_and_discharged": summary["translation_settled_and_discharged"],
            "total_kinetic_stable": summary["total_kinetic_stable"],
            "final_translation_J": translation, "final_rotation_J": rotation,
            "z_spin_fraction_of_rotation": float(np.sum(omega[:, 2]**2)/spin_square) if spin_square else 0,
            "loaded_outside_hopper": loaded_outside, "profile_sectors": len(angles),
            "profile_mean_angle_deg": mean,
            "spatial_sector_sd_deg": float(np.std([a["angle_deg"] for a in angles], ddof=1)) if len(angles)>1 else None,
            "minimum_profile_r_squared": min((a["r_squared"] for a in angles), default=None),
            "maximum_height_in_grain_diameters": float((xyz[:, 2].max()+diameter/2)/diameter),
            "raw_frames": len(frames), "raw_dump_sha256": raw_hash,
            "source_sha256": hashlib.sha256((folder/"source.py").read_bytes()).hexdigest(),
            "formal_calibration_pass": False}


def plots(root, rows):
    # Profile availability is not an accuracy gate; show poor fits alongside good ones.
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    baseline = root/"funnel32g"
    if (baseline/"summary.json").exists():
        protocol = json.loads((baseline/"protocol.json").read_text())
        loaded = np.load(baseline/"loaded_hopper.npz")["xyz"]*1000
        final = np.load(baseline/"final_particles.npz")["xyz"]*1000
        bottom, top = protocol["outlet_height_m"]*1000, protocol["funnel_top_m"]*1000
        axes[0, 0].scatter(loaded[:, 0], loaded[:, 2], s=1, alpha=0.4, label="Loaded, 32 g")
        axes[0, 0].scatter(final[:, 0], final[:, 2], s=1, alpha=0.4, label="Discharged, 32 g")
        for sign in (-1, 1):
            axes[0, 0].plot([sign*8, sign*60], [bottom, top], color="black")
        axes[0, 0].plot([-8, 8], [bottom, bottom], "r--", label="Removable gate")
        axes[0, 0].axhline(0, color="gray")
        axes[0, 0].set(xlabel="x (mm)", ylabel="z (mm)", title="Engineered funnel; 32 g x-z projection")
        axes[0, 0].set_aspect("equal", adjustable="datalim")
        axes[0, 0].legend(fontsize=8)
    profile_rows = [r for r in rows if r["profile_mean_angle_deg"] is not None]
    axes[0, 1].barh([r["trial"] for r in profile_rows], [r["profile_mean_angle_deg"] for r in profile_rows], color="#2478a5")
    axes[0, 1].set(xlabel="Computed outline angle (deg)", title="Single-seed diagnostics; not calibrated AoR")
    axes[0, 1].tick_params(axis="y", labelsize=8)
    for name in ("funnel32g", "funnel32g_half", "funnel200g"):
        folder = root/name
        if not (folder/"time_stats.csv").exists() or not (folder/"summary.json").exists():
            continue
        stats = list(csv.DictReader((folder/"time_stats.csv").open()))
        times = [float(s["time_s"]) for s in stats]
        axes[1, 0].semilogy(times, [max(float(s["kinetic_to_mgd"]), 1e-15) for s in stats], label=name+" translation")
        axes[1, 0].semilogy(times, [max(float(s["total_kinetic_to_mgd"]), 1e-15) for s in stats], linestyle=":", label=name+" total")
        summary = json.loads((folder/"summary.json").read_text())
        for a in summary["angles"]:
            profile = np.asarray(a["outline"])*1000
            axes[1, 1].plot(profile[:, 0], profile[:, 1], alpha=0.6)
    axes[1, 0].axhline(1e-5, color="gray", linestyle="--")
    axes[1, 0].set(xlabel="Simulation time (s)", ylabel="KE / (M g d)", title="Translation stop vs residual spin")
    axes[1, 0].legend(fontsize=7)
    axes[1, 1].set(xlabel="Radius (mm)", ylabel="Upper outline z (mm)", title="Completed 32 g spatial profiles: 8 / 4 us")
    for axis in axes.flat:
        axis.grid(alpha=0.15)
    fig.suptitle("Fixed-funnel apparatus checks | Unknown paper dimensions | No friction fitting", fontsize=13)
    fig.savefig(root/"apparatus_checks.png", dpi=160)
    plt.close(fig)


def main():
    root = Path(__file__).resolve().parent
    plan = json.loads((root/"plan.json").read_text())
    slots, rows = [], []
    for case in plan["cases"]:
        name = case["name"]
        folder = root/name
        if (root/(name+".interrupted.json")).exists():
            if not (root/(name+".exit")).exists():
                slots.append({"trial": name, "status": "pending_interruption"})
                continue
            with zipfile.ZipFile(folder/"particles.zip") as archive:
                assert archive.testzip() is None
                with archive.open("particles.dump") as stream:
                    raw_hash = digest(stream)
            slots.append({"trial": name, "status": "interrupted_for_compute_switch",
                          "exit_code": int((root/(name+".exit")).read_text()), "raw_dump_sha256": raw_hash})
            continue
        ready = (root/(name+".exit")).exists() or name == "smoke2g" and (folder/"particles.zip").exists()
        if not ready:
            slots.append({"trial": name, "status": "pending"})
            continue
        exit_code = int((root/(name+".exit")).read_text()) if (root/(name+".exit")).exists() else None
        if exit_code not in (0, None):
            slots.append({"trial": name, "status": "failed", "exit_code": exit_code})
            continue
        row = audit_trial(folder, case)
        rows.append(row)
        slots.append({"trial": name, "status": "raw_data_audited", "exit_code": exit_code})
    report = {"slots": slots, "rows": rows,
              "all_slots_finished": all(not s["status"].startswith("pending") for s in slots),
              "all_slots_audited": len(rows) == len(plan["cases"]),
              "paper_replication": False, "parameter_fit_performed": False,
              "limits": plan["limits"]+["Early geometry_measurement_usable fields mean availability only, not accuracy acceptance"]}
    (root/"audit.json").write_text(json.dumps(report, indent=2))
    if rows:
        with (root/"comparison.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        plots(root, rows)
    print(json.dumps({"slots": slots, "measurements": [{k:r[k] for k in ("trial", "profile_mean_angle_deg", "minimum_profile_r_squared", "total_kinetic_stable")} for r in rows]}, indent=2))


if __name__ == "__main__":
    main()
