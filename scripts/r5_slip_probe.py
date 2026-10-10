"""Authorized test-only WheelSlip intervention and read-only checks."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from audit_r5_drive_constraints import model_audit, WHEELS
from r5_load_diagnostic import semantic, diagnose_round


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def generate(source, output, authorization):
    auth = json.loads(Path(authorization).read_text())
    if auth.get('human_reply') != '允许测试侧向滑移模型，仅限测试副本' or auth.get('production_change_allowed'):
        raise ValueError('Explicit test-only authorization required')
    output = Path(output); output.mkdir()
    original = ET.parse(source); candidate = deepcopy(original)
    audit, _ = model_audit(source)
    plugin = ET.SubElement(candidate.find('model'), 'plugin',
        filename='gz-sim-wheel-slip-system', name='gz::sim::systems::WheelSlip')
    for name, settings in audit['proposal']['parameters'].items():
        wheel = ET.SubElement(plugin, 'wheel', link_name=name+'_wheel_link')
        for key, value in settings.items():
            ET.SubElement(wheel, key).text = format(value, '.17g')
    restored = deepcopy(candidate); restored.find('model').remove(restored.find("model/plugin[@name='gz::sim::systems::WheelSlip']"))
    if semantic(original.getroot()) != semantic(restored.getroot()):
        raise ValueError('Unexpected physical/model change')
    for name in ('baseline','restore'):
        (output/(name+'.sdf')).write_bytes(Path(source).read_bytes())
    candidate.write(output/'candidate.sdf')
    for name in ('baseline','candidate','restore'):
        (output/(name+'.urdf')).write_bytes(Path(source).with_suffix('.urdf').read_bytes())
    manifest = dict(single_change='Add speed-dependent native WheelSlip',
                    production_changed=False, contact_physics_changed_in_test_copy=True,
                    prediction=audit['proposal_prediction'], parameters=audit['proposal']['parameters'],
                    models={n:sha(output/(n+'.sdf')) for n in ('baseline','candidate','restore')})
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def slip_commands(data, result, profile, parameters):
    active = data[(data['t'] >= result['sim_started']) & (data['t'] <= result['sim_ended'])]
    checks = {}
    for w in WHEELS:
        present = active[w+'_slip_present']
        if profile == 'candidate':
            settings = parameters[w]
            expected = abs(active[w+'_spin_before'])*settings['wheel_radius']/settings['wheel_normal_force']*settings['slip_compliance_lateral']
            observed = active[w+'_slip1_cmd']
            error = abs(observed-expected)
            good = bool(len(active)>1 and np.isfinite(expected).all() and np.isfinite(error).all()
                        and (present == 1).all() and error.max() <= 1e-12
                        and np.isfinite(active[w+'_slip2_cmd']).all() and (active[w+'_slip2_cmd'] == 0).all())
            checks[w] = dict(passed=good, maximum_formula_error=float(error.max()) if np.isfinite(error).all() else None,
                             nonzero_steps=int(np.count_nonzero(observed>1e-8)))
        else:
            checks[w] = dict(passed=bool(len(active)>1 and (present==0).all()),
                             component_absent=bool((present==0).all()))
    return dict(passed=all(v['passed'] for v in checks.values()), wheels=checks,
                meaning='Read-only command observation; backend effect requires independent contact relation')


def contact_relation(folder, result, profile, parameters):
    folder=Path(folder)
    # Old load hypothesis fields are retained but never used as WheelSlip gates.
    diagnosis=diagnose_round(folder, {'center_of_mass_m':[-.006237889179185716,0,.12798331700413673]}, 'original')
    data=np.genfromtxt(folder/'physics-steps.csv', delimiter=',', names=True)
    start=result['schedule']['motion']['before']['sim']+3
    end=result['schedule']['motion']['after']['sim']-1
    relations={}
    for w in WHEELS:
        c=np.genfromtxt(folder/'load-analysis'/(w+'-contact-series.csv'),delimiter=',',names=True)
        c=c[(c['t']>=start)&(c['t']<=end)]
        if len(c)<2 or not np.isfinite(c['slip_body_y']).all() or not np.isfinite(c['force_body_y']).all():
            raise ValueError('Missing actual contact force/slip')
        cmd=np.interp(c['t'],data['t'],data[w+'_slip1_cmd'])
        predicted=-c['force_body_y']*cmd
        rms=float(np.sqrt(np.mean((c['slip_body_y']-predicted)**2))) if profile=='candidate' else None
        observed_rms=float(np.sqrt(np.mean(c['slip_body_y']**2)))
        relations[w]=dict(contact_samples=len(c),lateral_speed_rms_m_s=observed_rms,
                          force_compliance_residual_rms_m_s=rms,
                          backend_relation_passed=bool(rms is not None and rms<=.005 and observed_rms>=.005))
    load_qualified=bool(diagnosis['com_error_m']<=.0001 and
        abs(diagnosis['static_front_N']+diagnosis['static_rear_N']-diagnosis['weight_N'])<=.05*diagnosis['weight_N'])
    if result['origin_parameters']['angular']==0 and result['origin_parameters']['linear']==0:
        load_qualified=load_qualified and diagnosis['zero_stability_passed']
    return dict(contacts_qualified=load_qualified,icr_body_x_median_m=diagnosis['icr_body_x_median_m'],
                relations=relations,backend_effect_verified=bool(profile=='candidate' and all(v['backend_relation_passed'] for v in relations.values())),
                old_load_hypothesis_not_used_for_slip_verdict=True)


def compare(old, new):
    a=old['slip_probe']['trajectory']; b=new['slip_probe']['trajectory']
    return dict(maximum_excursion_reduction_m=a['maximum_displacement_m']-b['maximum_displacement_m'],
                remaining_maximum_excursion_m=b['maximum_displacement_m'],
                yaw_loss_rad=abs(a['unwrapped_turn_rad'])-abs(b['unwrapped_turn_rad']),
                mechanism_probe_passed=bool(b['maximum_displacement_m']<=.03 and
                    abs(a['unwrapped_turn_rad'])-abs(b['unwrapped_turn_rad'])<=.05 and
                    new['slip_probe']['contact']['backend_effect_verified']),
                previous_one_cm_screen_passed=b['maximum_displacement_m']<=.01,
                whole_vehicle_accepted=False)
