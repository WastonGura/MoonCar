"""Literature-model projection, not a fit to measured R5 wheel/soil data.

Ding et al. 2024, doi:10.1038/s41467-024-46233-8, equations 1,5,6,9,11,16,17.
The smooth-cylinder reduction h=0, s=0 and nominal soil point are declared
transfer assumptions. No whole-vehicle displacement enters this calculation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
integrate = getattr(np, 'trapezoid', np.trapz)


def root_bisection(function, lo, hi, tolerance=1e-14):
    if not function(lo) < 0 < function(hi):
        raise ValueError('Support root is not bracketed')
    for _ in range(100):
        mid = (lo+hi)/2
        if function(mid) > 0:
            hi = mid
        else:
            lo = mid
        if hi-lo <= tolerance:
            return (lo+hi)/2
    raise ValueError('Support root did not converge')


def minimize_bounded(function, lo, hi):
    ratio = (math.sqrt(5)-1)/2
    a, b = hi-ratio*(hi-lo), lo+ratio*(hi-lo)
    fa, fb = function(a), function(b)
    for _ in range(120):
        if fa < fb:
            hi, b, fb = b, a, fa
            a = hi-ratio*(hi-lo)
            fa = function(a)
        else:
            lo, a, fa = a, b, fb
            b = lo+ratio*(hi-lo)
            fb = function(b)
        if hi-lo < 1e-11:
            return (lo+hi)/2
    raise ValueError('Reference projection did not converge')

SOIL = dict(Ks_Pa_per_m=1443000., n=1., cohesion_Pa=520.,
            external_mu=.2, lateral_shear_length_m=.0265)
FIT_BETA_DEG = [.25, .5, 1., 1.5, 2., 2.5, 3.]
HOLDOUT_BETA_DEG = [4., 5., 7.5, 10.]


def reference(load_N, radius_m=.076, width_m=.05, soil=None, points=4097):
    """Solve static support, then integrate the paper's lateral shear law."""
    soil = SOIL if soil is None else soil
    if not all(math.isfinite(v) and v > 0 for v in
               (load_N, radius_m, width_m, soil['Ks_Pa_per_m'], soil['n'],
                soil['cohesion_Pa'], soil['external_mu'], soil['lateral_shear_length_m'])):
        raise ValueError('Positive finite reference inputs required')
    if points < 257 or points % 2 != 1:
        raise ValueError('Odd resolved integration grid required')

    def profile(z):
        entry = math.acos((radius_m-z)/radius_m)
        theta = np.linspace(0., entry, points)
        middle = entry/2
        # Equation 6, theta2=0, theta_m=theta1/2.
        effective = np.where(theta >= middle, theta, entry-theta)
        sigma = soil['Ks_Pa_per_m'] * radius_m**soil['n'] * np.maximum(
            0., np.cos(effective)-math.cos(entry))**soil['n']
        normal = radius_m*width_m*integrate(sigma*np.cos(theta), theta)
        return entry, theta, sigma, normal

    z = root_bisection(lambda d: profile(d)[3]-load_N, 1e-9, radius_m*.95)
    entry, theta, sigma, normal = profile(z)
    strength = soil['cohesion_Pa']+sigma*soil['external_mu']
    # Effective cap includes cohesion and contact geometry. It is NOT tan(phi_y).
    cap = radius_m*width_m*integrate(strength, theta)/load_N

    def force_ratio(beta_deg):
        beta = np.asarray(beta_deg, dtype=float)
        if not np.isfinite(beta).all() or np.any(abs(beta) >= 89):
            raise ValueError('Finite slip angles inside (-89,89) required')
        slip = np.tan(np.radians(abs(beta)))
        jy = radius_m*(entry-theta)[None, :]*slip.reshape(-1, 1)
        tau = strength[None, :]*(-np.expm1(-jy/soil['lateral_shear_length_m']))
        result = radius_m*width_m*integrate(tau, theta, axis=1)/load_N
        return (result*np.sign(beta).reshape(-1)).reshape(beta.shape)

    return dict(load_N=load_N, sinkage_m=z, normal_integral_N=normal,
                effective_lateral_cap=cap, entry_angle_rad=entry), force_ratio


def gazebo_ratio(beta_deg, compliance, cap):
    if not math.isfinite(compliance) or compliance <= 0 or not math.isfinite(cap) or cap <= 0:
        raise ValueError('Positive finite surrogate parameters required')
    beta = np.asarray(beta_deg, dtype=float)
    # lambda_y = vy/(r*abs(omega)) = tan(beta) for reference s=0.
    return np.clip(np.tan(np.radians(beta))/compliance, -cap, cap)


def fit(load_N, soil=None):
    soil = SOIL if soil is None else soil
    geometry, target = reference(load_N, soil=soil)
    x = np.array(FIT_BETA_DEG)
    y = target(x)
    cap = geometry['effective_lateral_cap']
    opt = minimize_bounded(lambda logc: float(np.mean(
        (gazebo_ratio(x, math.exp(logc), cap)-y)**2)),
        math.log(.001), math.log(1000.))
    compliance = math.exp(opt)

    def rows(angles):
        ref = target(angles)
        sim = gazebo_ratio(angles, compliance, cap)
        return [dict(beta_deg=b, reference_force_ratio=float(a),
                     surrogate_force_ratio=float(s), absolute_error=float(abs(a-s)),
                     relative_error=float(abs(a-s)/abs(a)))
                for b, a, s in zip(angles, ref, sim)]

    training, holdout = rows(FIT_BETA_DEG), rows(HOLDOUT_BETA_DEG)
    # Fixed beforehand; fit and independent angle extrapolation reported separately.
    training_passed = max(r['absolute_error'] for r in training) <= .01
    holdout_passed = max(r['absolute_error'] for r in holdout) <= .03
    return dict(**geometry, compliance_lateral=compliance, soil=soil,
                training=training, angle_holdout=holdout,
                training_passed=training_passed, angle_holdout_passed=holdout_passed,
                hardware_calibrated=False, measured_force_data_used=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline-model', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    from audit_r5_mass import aggregate, read_sdf
    from audit_r5_drive_constraints import static_loads
    import xml.etree.ElementTree as ET
    tree = ET.parse(a.baseline_model)
    model = tree.find("world/model[@name='mooncar']")
    if model is None:
        raise ValueError('Frozen baseline vehicle required')
    standalone = ET.Element('sdf', version='1.10')
    import copy
    standalone.append(copy.deepcopy(model))
    combined = aggregate(read_sdf(standalone))
    loads = static_loads(combined['total_mass_kg'], combined['com_m'][0], .157, -.157, 1.62)
    axles = {axle: fit(load/2) for axle, load in loads.items()}
    # Source uncertainty is a model envelope, not a confidence interval. These
    # bounding combinations are sensitivity scenarios, not identified soil pairs.
    envelope = []
    for ky in (.013, .04):
        for mu in (math.tan(math.radians(8.3)), math.tan(math.radians(16.5))):
            soil = dict(SOIL, external_mu=mu, lateral_shear_length_m=ky)
            envelope.append(dict(soil=soil, front=fit(loads['front']/2, soil),
                                 rear=fit(loads['rear']/2, soil)))
    result = dict(kind='literature_model_projection_not_empirical_calibration',
        doi='10.1038/s41467-024-46233-8', equations=[1,5,6,9,11,16,17],
        reference_assumptions=dict(smooth_cylinder=True, lug_height_m=0,
            longitudinal_slip=0, side_bulldozing_omitted=True,
            nominal_soil_point_is_engineering_selection=True),
        baseline_model_sha256=hashlib.sha256(Path(a.baseline_model).read_bytes()).hexdigest(),
        reference_fit_limit_abs_force_ratio=.01, angle_holdout_limit_abs_force_ratio=.03,
        total_mass_kg=combined['total_mass_kg'], com_m=combined['com_m'],
        axles=axles, sensitivity_envelope=envelope,
        baseline_compliance_lateral=10., longitudinal_compliance_unchanged=0.,
        production_changed=False, observed_whole_vehicle_data_used_in_fit=False)
    out = Path(a.output)
    if out.exists():
        raise FileExistsError('Do not overwrite fitted evidence')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: {n: v[n] for n in ['load_N','sinkage_m','effective_lateral_cap',
        'compliance_lateral','training_passed','angle_holdout_passed']} for k,v in axles.items()}))


if __name__ == '__main__':
    main()
