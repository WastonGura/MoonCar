"""Independent DEM measurement prototype, not an EDEM replication or soil fit."""

import argparse
import csv
import json
from math import atan, degrees, pi, sqrt
from pathlib import Path
import time
import zipfile

import numpy as np
from lammps import lammps


DIAMETER = 0.002
DENSITY = 2650.0
YOUNG = 5e7
POISSON = 0.25
GRAVITY = 9.81


def contact(mu, rolling, gamma=0):
    # JKR forbids coeff_restitution; its smoke test uses a 50/s mass-velocity damping input.
    normal = f"jkr {YOUNG} 50 {POISSON} {gamma}" if gamma else f"hertz/material {YOUNG} 0.3 {POISSON}"
    damping = "mass_velocity" if gamma else "coeff_restitution"
    # ponytail: SDS stiffness is an engineering input; validate against Type-C before parameter transfer.
    return normal + f" tangential mindlin NULL 1.0 {mu} rolling sds 1000 0 {rolling} damping {damping}"


def self_check():
    results = {}
    for name, overlap, gamma in (("hertz", 1e-5, 0), ("separated", -1e-4, 0), ("jkr", 1e-9, 0.05)):
        engine = lammps(cmdargs=["-screen", "none", "-log", "none"])
        try:
            engine.commands_string(f"""units si
atom_style sphere
boundary f f f
region box block -0.01 0.01 -0.01 0.01 -0.01 0.01
create_box 1 box
create_atoms 1 single 0 0 0
create_atoms 1 single {DIAMETER-overlap} 0 0
set group all diameter {DIAMETER}
set group all density {DENSITY}
comm_modify vel yes
pair_style granular
pair_coeff * * {contact(0.65, 0.15, gamma)}
run 0
""")
            forces = engine.numpy.extract_atom("f")[:2].copy()
            assert np.allclose(forces[0]+forces[1], 0, atol=1e-12)
            force = float(forces[1, 0])
            if name == "hertz":
                expected = 4/3 * (YOUNG/(2*(1-POISSON**2))) * sqrt(DIAMETER/4) * overlap**1.5
                assert abs(force/expected-1) < 1e-8, (force, expected)
                results["hertz_analytic_force_N"] = expected
            elif name == "separated":
                assert abs(force) < 1e-12
            else:
                assert force < 0, force
            results[name+"_force_N"] = force
            results["lammps_version"] = engine.version()
        finally:
            engine.close()
    # Check the outline estimator with an exact, sampled cone, independently of DEM.
    rng = np.random.default_rng(123)
    radii = rng.uniform(0.002, 0.05, 20000)
    azimuth = rng.uniform(0, 2*pi, len(radii))
    xyz = np.column_stack((radii*np.cos(azimuth), radii*np.sin(azimuth), (0.05-radii)*np.tan(np.radians(34))))
    angles = repose_angles(xyz, radius=0)
    assert len(angles) == 8 and all(abs(row["angle_deg"]-34) < 1.5 for row in angles), angles
    results["outline_cone_check"] = True
    return results


def repose_angles(xyz, radius=DIAMETER/2):
    """Eight radial upper outlines; exclude the inner apex and outer toe."""
    radial = np.hypot(xyz[:, 0], xyz[:, 1])
    sector = ((np.arctan2(xyz[:, 1], xyz[:, 0])+pi)/(2*pi)*8).astype(int) % 8
    rows = []
    for index in range(8):
        mask = sector == index
        if mask.sum() < 30:
            continue
        extent = float(np.quantile(radial[mask], 0.95))
        edges = np.arange(0.2*extent, 0.8*extent+0.001, DIAMETER)
        profile = []
        for left, right in zip(edges[:-1], edges[1:]):
            chosen = mask & (radial >= left) & (radial < right)
            if chosen.sum() >= 3:
                profile.append(((left+right)/2, float(xyz[chosen, 2].max()+radius)))
        if len(profile) < 4:
            continue
        x, z = np.asarray(profile).T
        slope, intercept = np.polyfit(x, z, 1)
        residual = float(np.sum((z-(slope*x+intercept))**2))
        variance = float(np.sum((z-z.mean())**2))
        rows.append({"sector": index, "angle_deg": degrees(atan(-slope)),
                     "r_squared": 1-residual/variance if variance else 0,
                     "profile_bins": len(profile), "fit_extent_m": extent,
                     "outline": profile})
    return rows


def run(args):
    assert 0 <= args.mu <= 2 and 0 <= args.rolling <= 1
    assert 0 < args.mass_g <= 200 and args.seconds > 0 and args.dt > 0
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    checks = self_check()
    (out/"checks.json").write_text(json.dumps(checks, indent=2))
    particle_mass = DENSITY*pi/6*DIAMETER**3
    count = round(args.mass_g/1000/particle_mass)
    interval = round(0.08/args.dt)
    assert count >= 100 and interval >= 1
    coefficients = contact(args.mu, args.rolling)
    commands = f"""units si
atom_style sphere
boundary m m f
newton off
region domain block -0.15 0.15 -0.15 0.15 -0.005 0.25 units box
create_box 1 domain
comm_modify vel yes
neighbor 0.0005 bin
neigh_modify delay 0 every 1 check yes
pair_style granular
pair_coeff * * {coefficients}
fix floor all wall/gran granular {coefficients} zplane 0 NULL
fix integrator all nve/sphere
fix gravity all gravity {GRAVITY} vector 0 0 -1
region outlet cylinder z 0 0 0.008 0.10 0.13 units box
fix feed all pour {count} 1 {args.seed} region outlet diam one {DIAMETER} dens {DENSITY} {DENSITY} vol 0.25 100
timestep {args.dt}
compute kinetic all ke
thermo {interval}
thermo_style custom step time atoms c_kinetic
thermo_modify lost error
dump trajectory all custom {interval} {out/'particles.dump'} id type x y z vx vy vz radius
dump_modify trajectory sort id
"""
    (out/"input.lmp").write_text(commands)
    protocol = {"role": "functional prototype; not a calibrated soil or paper reproduction",
                "normal_model": "Hertz-Mindlin, noncohesive Toyoura baseline",
                "rolling_model": "LAMMPS SDS; not proven equivalent to EDEM Type-C",
                "diameter_m": DIAMETER, "density_kg_m3": DENSITY, "young_Pa": YOUNG,
                "poisson": POISSON, "restitution": 0.3, "particle_mu": args.mu,
                "rolling_mu": args.rolling, "rolling_stiffness": 1000, "rolling_damping": 0,
                "dt_s": args.dt, "seed": args.seed, "target_particles": count,
                "target_mass_kg": count*particle_mass, "max_time_s": args.seconds,
                "outlet_radius_m": 0.008, "outlet_bottom_m": 0.10, "outlet_top_m": 0.13,
                "horizontal_boundary": "shrink-wrap minimum; unbounded support plane",
                "geometry_status": "engineering emitter, physical funnel geometry not yet verified",
                "repose_target_Toyoura_deg": 34.3, "paper_reported_error_deg": 0.8,
                "target_used_for_fitting": False}
    (out/"protocol.json").write_text(json.dumps(protocol, indent=2))
    engine = lammps(cmdargs=["-screen", "none", "-log", str(out/"lammps.log")])
    started = time.monotonic()
    stats = []
    settled = False
    try:
        engine.commands_string(commands)
        for block in range(int(args.seconds/args.dt)//interval):
            engine.command(f"run {interval} pre yes post no")
            actual = int(engine.get_natoms())
            xyz = engine.numpy.extract_atom("x")[:actual].copy()
            velocity = engine.numpy.extract_atom("v")[:actual].copy()
            energy = float(engine.extract_compute("kinetic", 0, 0))
            ratio = energy/(actual*particle_mass*GRAVITY*DIAMETER) if actual else 0
            stats.append({"time_s": float(engine.get_thermo("time")), "particles": actual,
                          "kinetic_J": energy, "kinetic_to_mgd": ratio,
                          "max_speed_m_s": float(np.linalg.norm(velocity, axis=1).max()) if actual else 0})
            if actual == count and len(stats) >= 3 and all(row["kinetic_to_mgd"] < 1e-5 for row in stats[-3:]):
                settled = True
                break
        engine.command("undump trajectory")
        np.savez_compressed(out/"final_particles.npz", xyz=xyz, velocity=velocity)
        angles = repose_angles(xyz)
        summary = {"inserted_particles": actual, "expected_particles": count,
                   "mass_conserved": actual == count, "settled_by_kinetic_check": settled,
                   "wall_seconds": time.monotonic()-started, "sim_seconds": stats[-1]["time_s"],
                   "angles": angles,
                   "mean_angle_deg": float(np.mean([row["angle_deg"] for row in angles])) if angles else None,
                   "minimum_profile_r_squared": min((row["r_squared"] for row in angles), default=None),
                   "formal_calibration_pass": False}
        (out/"summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps({key: value for key, value in summary.items() if key != "angles"}), flush=True)
        assert actual == count, "Insertion did not complete; retain evidence, do not report an angle as validated"
    except Exception as error:
        (out/"failed.json").write_text(json.dumps({"error": str(error), "formal_calibration_pass": False}, indent=2))
        raise
    finally:
        engine.close()
        if stats:
            with (out/"time_stats.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=stats[0].keys())
                writer.writeheader()
                writer.writerows(stats)
        if (out/"particles.dump").exists():
            with zipfile.ZipFile(out/"particles.zip", "w", zipfile.ZIP_DEFLATED) as archive:
                archive.write(out/"particles.dump", "particles.dump")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--out", default="prototype")
    parser.add_argument("--mass-g", type=float, default=32)
    parser.add_argument("--seconds", type=float, default=4)
    parser.add_argument("--dt", type=float, default=8e-6)
    parser.add_argument("--mu", type=float, default=0.65)
    parser.add_argument("--rolling", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=123457)
    args = parser.parse_args()
    if args.check:
        print(json.dumps(self_check(), indent=2))
    else:
        run(args)
