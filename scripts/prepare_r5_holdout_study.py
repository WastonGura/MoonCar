"""Pre-register a frozen-candidate validation; never fit or publish defaults."""
from pathlib import Path
import datetime,hashlib,json,shutil

ROOT=Path(__file__).resolve().parents[1]
E=ROOT/'reports/evidence/terrain-v2/r5-frozen-holdout-20261018'
OLD=E.parent/'r5-integral-contact-20261018'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()

def main():
    E.mkdir(exist_ok=False)
    for d in ('tools','runtime','plugin','sources','native'):(E/d).mkdir()
    for name in ('reference-profile.json','candidate_profile.json','front-law.txt','rear-law.txt'):
        shutil.copyfile(OLD/name,E/name)
    shutil.copyfile(OLD/'plugin/libmooncar-integral-contact.so',E/'plugin/libmooncar-integral-contact.so')
    frozen={str(p.relative_to(ROOT)):sha(p) for p in [ROOT/'scripts/r5_integral_contact.py',ROOT/'containers/r5_integral_contact/Law.hh',ROOT/'containers/r5_integral_contact/IntegralContact.cc',ROOT/'containers/r5_turn_runtime/GatePolicy.hh',OLD/'reference-profile.json',OLD/'candidate_profile.json',OLD/'front-law.txt',OLD/'rear-law.txt',OLD/'plugin/libmooncar-integral-contact.so']}
    original=json.loads((OLD/'baseline-before.json').read_text(encoding='utf-8-sig'))
    for name in original:
        original[name]=sha(ROOT/name)
    (E/'baseline-before.json').write_text(json.dumps(original,indent=2)+'\n')
    (E/'candidate-freeze.json').write_text(json.dumps(dict(created=datetime.datetime.now(datetime.timezone.utc).isoformat(),files=frozen,production_default=False,refitting_allowed=False),indent=2)+'\n')
    conditions=[dict(id='flat-slow-long',scene='flat',angular=.08,duration_s=60,slope_deg=0,azimuth_deg=0),
                dict(id='flat-fast-reverse',scene='flat',angular=-.32,duration_s=30,slope_deg=0,azimuth_deg=0),
                dict(id='plane3-oblique',scene='plane',angular=.13,duration_s=30,slope_deg=3,azimuth_deg=60),
                dict(id='plane5-reverse',scene='plane',angular=-.27,duration_s=30,slope_deg=5,azimuth_deg=145)]
    rows=[]
    for c in conditions:
        for variant in ('baseline','candidate'):
            rows.append(dict(c,name=f'{len(rows)+1:02d}-{c["id"]}-{variant}',variant=variant,step=.00025))
    for index in (3,7):
        s=dict(rows[index]);s.update(name=f'{len(rows)+1:02d}-{s["id"]}-candidate-coarse',step=.0005);rows.append(s)
    for index in (4,5):
        s=dict(rows[index]);s.update(name=f'{len(rows)+1:02d}-{s["id"]}-{s["variant"]}-repeat');rows.append(s)
    native=[]
    for axle in ('front','rear'):
        for speed in (.006,.018,.075):
            for beta in (7.3,37.1,82.4):
                native.append(dict(name=f'{axle}-u{speed}-b{beta}',axle=axle,rolling=speed,beta=beta,sign=1 if beta!=37.1 else -1,load_scale=1.,step=.00025))
        for load in (.7,1.3):
            native.append(dict(name=f'{axle}-load{load}',axle=axle,rolling=.018,beta=37.1,sign=-1,load_scale=load,step=.00025))
        native.append(dict(name=f'{axle}-coarse',axle=axle,rolling=.075,beta=82.4,sign=1,load_scale=1.,step=.0005))
    protocol=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        purpose='Frozen candidate prospective new operating combinations; not empirical calibration or unseen reference formula',
        candidate_freeze_sha256=sha(E/'candidate-freeze.json'),no_fit_or_model_selection=True,
        previously_seen='Flat/1.579735deg plane/old wavy mesh; angular +/-0.2rad/s; 20s motion. Dense reference angle grids already seen.',
        novel='New rate-duration combinations and newly generated 3deg/60deg and 5deg/145deg slope meshes; frozen before any new physics result.',
        rounds=rows,maximum_attempts=len(rows),native_cases=native,
        native_rig='DART prescribed lateral velocity slider, frozen IntegralLaw and cap; normal load variation is a test input only; not an actual wheel in soil',
        native_duration_s=4,native_settle_s=.5,native_steady_window_s=[3,4],
        force_gates=dict(reference_relative_error_max=.01,native_force_vs_frozen_law_relative_max=.01,normal_support_relative_max=.01,contact_power_positive_max_W=1e-8,
                         load_extrapolation_vs_recomputed_reference_relative_max=.10,
                         absolute_error_floor_N=1e-6,
                         empirical_force_error='NOT AVAILABLE until compatible real measurements are acquired; never pass from simulation'),
        schedule=dict(parked_s=5,stop_s=3,warmup='unchanged readiness loop'),
        data_qualification='All original strict native library, physics, contact, image, DDS, TF, actual command, flush, drain and cleanup checks; only duration binding and exact new-plane ground reference adapted',
        engineering=dict(max_displacement_m=.5,max_abs_roll_pitch_deg=20,max_steady_empty_contact_fraction=.05,max_suspension_abs_m=.02001,max_stopped_speed_m_s=.001,
                         turn_direction='same sign as command',turn_budget_ratio_range=[.5,1.5]),
        rotation=dict(window='motion+5s through motion_end-1s',fit='p=c_world-R(yaw)r_body+v_world*(t-tmean)',
                      valid_fit_rms_mm=1.,valid_fit_condition_max=10000,minimum_yaw_span_rad=1.,
                      flat_max_icr_offset_mm=1.,plane_max_icr_offset_mm=5.,flat_max_drift_mm_s=.01,plane_max_drift_mm_s=.10,
                      split='two equal time windows, each fit independently; report ICR/drift changes and RMS, no selective steady window',
                      invalid_fit='ICR and drift gates indeterminate, never pass; report instantaneous velocity ICR distribution separately'),
        convergence=dict(icr_difference_mm='max(0.05mm,5% fine offset)',drift_vector_difference_mm_s=.01,
                         displacement_difference_mm='max(2mm,5% fine displacement)',turn_relative_difference=.02),
        repeat=dict(position_max_difference_m=1e-6,orientation_quaternion_chord_max=1e-6,meaning='deterministic reproducibility, not statistical confidence'),
        legacy_1cm_screen='preserve original raw result; report separately',
        failure_policy='No retries, no refit, no thresholds changed after outcomes. Acquisition failures stop; physical performance gate failures retained and remaining planned cases continue.',
        maximum_wall_seconds_per_round=1800,automatic_retries=False,hardware_calibrated=False,production_changed=False)
    (E/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    (E/'README.md').write_text('# Frozen candidate new-condition validation\n\nStatus: preregistered, physics not started.\n\nCandidate and protocol are frozen separately. Empirical force error remains unavailable without suitable measured data.\n')
    print(json.dumps(dict(evidence=str(E),rounds=len(rows),native_cases=len(native),protocol_sha256=sha(E/'protocol.json'),frozen_files=len(frozen))))

if __name__=='__main__':main()
