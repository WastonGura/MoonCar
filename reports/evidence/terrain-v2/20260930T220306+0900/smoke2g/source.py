"""Engineering fixed-funnel DEM test; dimensions are not a paper replication."""

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


def contact_check():
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
    rolling_speeds = []
    for rolling in (0.0, 0.15):
        engine = lammps(cmdargs=["-screen", "none", "-log", "none"])
        try:
            coeff = contact(0.65, rolling)
            engine.commands_string(f"""units si
atom_style sphere
boundary f f f
region box block -0.05 0.05 -0.05 0.05 -0.005 0.05
create_box 1 box
create_atoms 1 single 0 0 0.001
set group all diameter {DIAMETER}
set group all density {DENSITY}
comm_modify vel yes
pair_style granular
pair_coeff * * {coeff}
fix floor all wall/gran granular {coeff} zplane 0 NULL
fix integrator all nve/sphere
fix gravity all gravity {GRAVITY} vector 0 0 -1
timestep 0.000008
run 5000
velocity all set 0.1 0 0
set group all omega 0 100 0
run 5000
""")
            rolling_speeds.append(float(engine.numpy.extract_atom("v")[0, 0]))
        finally:
            engine.close()
    assert abs(rolling_speeds[0]-0.1) < 0.005, rolling_speeds
    assert 0 <= rolling_speeds[1] < rolling_speeds[0]-0.01, rolling_speeds
    results["rolling_plane_vx_without_with_resistance_m_s"] = rolling_speeds
    return results


def self_check():
    results = contact_check()
    coeff = contact(0.65, 0.15)
    for name, x, z in (("cone_contact", 0.0335, 0.14), ("open_outlet", 0, 0.08)):
        engine = lammps(cmdargs=["-screen", "none", "-log", "none"])
        try:
            engine.commands_string(f"""units si
atom_style sphere
boundary f f f
region box block -0.15 0.15 -0.15 0.15 -0.005 0.25
create_box 1 box
create_atoms 1 single {x} 0 {z}
set group all diameter {DIAMETER}
set group all density {DENSITY}
set group all omega 0 100 0
comm_modify vel yes
pair_style granular
pair_coeff * * {coeff}
region funnel cone z 0 0 0.008 0.06 0.10 0.18 open 1 open 2 units box
fix funnel_wall all wall/gran/region granular {coeff} region funnel
compute rotation all erotate/sphere
thermo_style custom step atoms c_rotation
run 0
""")
            force = engine.numpy.extract_atom("f")[0].copy()
            if name == "cone_contact":
                assert force[0] < 0 and force[2] > 0, force
            else:
                assert np.linalg.norm(force) < 1e-12, force
            mass = DENSITY*pi/6*DIAMETER**3
            expected = 0.5*(0.4*mass*(DIAMETER/2)**2)*100**2
            actual = float(engine.extract_compute("rotation", 0, 0))
            assert abs(actual/expected-1) < 1e-10, (actual, expected)
            results[name+"_force_N"] = force.tolist()
            results["rotation_analytic_measured_J"] = [expected, actual]
        finally:
            engine.close()
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
    assert 0 <= args.wall_mu <= 2 and 0 <= args.wall_rolling <= 1
    assert 0.5 <= args.diameter_mm <= 2 and 0.05 <= args.outlet_height <= 0.15
    assert 0 < args.mass_g <= 200 and args.seconds > 0 and args.dt > 0
    diameter = args.diameter_mm/1000
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    (out/"source.py").write_bytes(Path(__file__).read_bytes())
    checks = self_check()
    (out/"checks.json").write_text(json.dumps(checks, indent=2))
    particle_mass = DENSITY*pi/6*diameter**3
    count = round(args.mass_g/1000/particle_mass)
    interval = round(0.08/args.dt)
    assert count >= 100 and interval >= 1
    coefficients = contact(args.mu, args.rolling)
    wall_coefficients = contact(args.wall_mu, args.wall_rolling)
    bottom, top = args.outlet_height, args.outlet_height+0.08
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
fix floor all wall/gran granular {wall_coefficients} zplane 0 NULL
region funnel cone z 0 0 0.008 0.06 {bottom} {top} open 1 open 2 units box
fix funnel_wall all wall/gran/region granular {wall_coefficients} region funnel
fix gate all wall/gran granular {wall_coefficients} zplane {bottom} NULL
fix integrator all nve/sphere
fix gravity all gravity {GRAVITY} vector 0 0 -1
region feed_zone cylinder z 0 0 0.04 {top+0.005} {top+0.025} units box
fix feed all pour {count} 1 {args.seed} region feed_zone diam one {diameter} dens {DENSITY} {DENSITY} vol 0.25 100
timestep {args.dt}
compute kinetic all ke
compute rotation all erotate/sphere
thermo {interval}
thermo_style custom step time atoms c_kinetic c_rotation
thermo_modify lost error
dump trajectory all custom {interval} {out/'particles.dump'} id type x y z vx vy vz omegax omegay omegaz radius
dump_modify trajectory sort id
"""
    (out/"input.lmp").write_text(commands)
    protocol = {"role": "engineering fixed-funnel test; not a calibrated soil or paper reproduction",
                "normal_model": "Hertz-Mindlin, noncohesive Toyoura baseline",
                "rolling_model": "LAMMPS SDS; not proven equivalent to EDEM Type-C",
                "diameter_m": diameter, "density_kg_m3": DENSITY, "young_Pa": YOUNG,
                "poisson": POISSON, "restitution": 0.3, "particle_mu": args.mu,
                "rolling_mu": args.rolling, "rolling_stiffness": 1000, "rolling_damping": 0,
                "wall_mu": args.wall_mu, "wall_rolling_mu": args.wall_rolling,
                "wall_parameter_basis": "engineering baseline; not measured sand-aluminium interface",
                "dt_s": args.dt, "seed": args.seed, "target_particles": count,
                "target_mass_kg": count*particle_mass, "max_time_s": args.seconds,
                "outlet_radius_m": 0.008, "outlet_height_m": bottom, "funnel_top_m": top,
                "funnel_top_radius_m": 0.06, "feed_radius_m": 0.04,
                "feed_bottom_m": top+0.005, "feed_top_m": top+0.025,
                "release_rule": "remove gate after complete insertion and 0.24 s additional loading time",
                "horizontal_boundary": "shrink-wrap minimum; unbounded support plane",
                "geometry_status": "engineering cone wall and removable gate; paper dimensions unknown",
                "settling_metric": "fully discharged and translational KE/(M*g*d)<1e-5 for three 0.08 s samples",
                "rotation_rule": "measured separately; total KE stability reported, not required for geometric stop",
                "profile_bin_m": DIAMETER,
                "repose_target_Toyoura_deg": 34.3, "paper_reported_error_deg": 0.8,
                "target_used_for_fitting": False}
    (out/"protocol.json").write_text(json.dumps(protocol, indent=2))
    engine = lammps(cmdargs=["-screen", "none", "-log", str(out/"lammps.log")])
    started = time.monotonic()
    stats = []
    settled = False
    release_time = None
    inserted_at = None
    try:
        engine.commands_string(commands)
        for block in range(int(args.seconds/args.dt)//interval):
            engine.command(f"run {interval} pre yes post no")
            with (out/"input.lmp").open("a") as stream:
                stream.write(f"run {interval} pre yes post no\n")
            actual = int(engine.get_natoms())
            xyz = engine.numpy.extract_atom("x")[:actual].copy()
            velocity = engine.numpy.extract_atom("v")[:actual].copy()
            omega = engine.numpy.extract_atom("omega")[:actual].copy()
            energy = float(engine.extract_compute("kinetic", 0, 0))
            rotation = float(engine.extract_compute("rotation", 0, 0))
            scale = actual*particle_mass*GRAVITY*diameter
            ratio = energy/scale if actual else 0
            now = float(engine.get_thermo("time"))
            remaining = int(np.sum(xyz[:, 2]+diameter/2 >= bottom))
            if actual == count and inserted_at is None:
                inserted_at = now
            if inserted_at is not None and release_time is None and now-inserted_at >= 0.24-1e-10:
                engine.command("unfix gate")
                with (out/"input.lmp").open("a") as stream:
                    stream.write(f"\n# At time {now}\nunfix gate\n")
                release_time = now
            stats.append({"time_s": now, "particles": actual,
                          "kinetic_J": energy, "kinetic_to_mgd": ratio,
                          "rotation_J": rotation, "total_kinetic_to_mgd": (energy+rotation)/scale if actual else 0,
                          "max_omega_rad_s": float(np.linalg.norm(omega, axis=1).max()) if actual else 0,
                          "particles_above_outlet": remaining, "gate_open": release_time is not None,
                          "max_speed_m_s": float(np.linalg.norm(velocity, axis=1).max()) if actual else 0})
            with (out/"progress.json").open("w") as stream:
                json.dump({"wall_seconds": time.monotonic()-started, **stats[-1]}, stream)
            if release_time is not None and actual == count and len(stats) >= 3 and all(
                row["particles_above_outlet"] == 0 and row["kinetic_to_mgd"] < 1e-5 for row in stats[-3:]
            ):
                settled = True
                break
        engine.command("undump trajectory")
        np.savez_compressed(out/"final_particles.npz", xyz=xyz, velocity=velocity, omega=omega)
        heap = xyz[xyz[:, 2]+diameter/2 < bottom]
        angles = repose_angles(heap, radius=diameter/2)
        summary = {"inserted_particles": actual, "expected_particles": count,
                   "mass_conserved": actual == count, "translation_settled_and_discharged": settled,
                   "particles_in_heap": len(heap), "particles_above_outlet": remaining,
                   "release_time_s": release_time,
                   "total_kinetic_stable": settled and all(row["total_kinetic_to_mgd"] < 1e-5 for row in stats[-3:]),
                   "measurement_usable": settled and len(angles) == 8,
                   "wall_seconds": time.monotonic()-started, "sim_seconds": stats[-1]["time_s"],
                   "angles": angles,
                   "final_translation_J": energy, "final_rotation_J": rotation,
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
    parser.add_argument("--mass-g", type=float, default=200)
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--diameter-mm", type=float, default=2)
    parser.add_argument("--outlet-height", type=float, default=0.10)
    parser.add_argument("--dt", type=float, default=8e-6)
    parser.add_argument("--mu", type=float, default=0.65)
    parser.add_argument("--rolling", type=float, default=0.15)
    parser.add_argument("--wall-mu", type=float, default=0.65)
    parser.add_argument("--wall-rolling", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=123457)
    args = parser.parse_args()
    if args.check:
        print(json.dumps(self_check(), indent=2))
    else:
        run(args)
