"""One declared diagnostic intervention using the complete real ROS fixture."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--name', required=True)
    args = parser.parse_args()
    manifest = json.loads((Path(args.prepared)/'manifest.json').read_text())
    spec = next(r for r in manifest['rounds'] if r['name'] == args.name)
    if hashlib.sha256(Path(spec['world']).read_bytes()).hexdigest() != spec['world_sha256']:
        raise ValueError('Declared world changed')
    # Adapt qualifications to the PREDECLARED intervention, without changing
    # the actual command path, input timing or acquisition thresholds.
    import r5_axle_allocation
    original_budget = r5_axle_allocation.qualify_budget
    def budget(active, path, linear, angular, enabled, ratios):
        return original_budget(active, path, linear, angular, spec['allocation_enabled'], ratios)
    r5_axle_allocation.qualify_budget = budget
    import r5_turn_slip_control
    original_slip = r5_turn_slip_control.qualify
    def slip(physics, result, path, mode, parameters):
        altered = copy.deepcopy(parameters)
        for value in altered.values():
            value['slip_compliance_lateral'] = spec['slip_compliance_lateral']
        return original_slip(physics, result, path, mode, altered)
    r5_turn_slip_control.qualify = slip
    import check_r5_coplanar_round
    old_argv = sys.argv
    try:
        sys.argv = ['diagnostic', '--output', args.output, '--name', args.name,
            '--world', spec['world'], '--ode-profile', 'candidate',
            '--candidate-runtime', '/opt/mooncar/turn/r5-v2',
            '--manifest', '/opt/mooncar/turn/r5-v2/manifest.json',
            '--round-fixture', manifest['fixture'], '--angular', str(spec['angular'])]
        code = check_r5_coplanar_round.main()
    finally:
        sys.argv = old_argv
        r5_axle_allocation.qualify_budget = original_budget
        r5_turn_slip_control.qualify = original_slip
    folder = Path(args.output)/args.name
    r = json.loads((folder/'result.json').read_text())
    actual = ET.parse(folder/'world.sdf')
    declared = ET.parse(spec['world'])
    # Ignore only observation output filenames and duplicate observer removal.
    def semantic(e):
        return e.tag, sorted(e.attrib.items()), (e.text or '').strip(), tuple(semantic(c) for c in e)
    def normalized(tree):
        root = copy.deepcopy(tree.getroot())
        model = root.find("world/model[@name='mooncar']")
        for p in model.findall('plugin'):
            if p.get('name') in ('mooncar::StepObserver', 'mooncar::ContactObserver'):
                model.remove(p)
            else:
                for output in p.findall('output'):
                    output.text = 'OBSERVATION_PATH'
        return semantic(root)
    same = normalized(actual) == normalized(declared)
    diagnostic = json.loads((folder/'diagnostic-result.json').read_text())
    loader = json.loads((Path(args.output)/(args.name+'-loader.json')).read_text())
    qualified = bool(diagnostic['eligible'] and loader['library_verified'] and not loader['error'] and same and code == 0)
    summary = dict(specification=spec, diagnostic_qualified=qualified,
        exact_declared_world_consumed=same, loader_verified=loader['library_verified'],
        existing_geometric_screen_passed=diagnostic['geometric_screen_passed'],
        original_result_and_screen_retained=True, vehicle_accepted=False,
        trajectory=r.get('turn_slip_probe', {}).get('trajectory'),
        all_checks=diagnostic['checks'], errors={k:r.get(k) for k in ('error','production_error','origin_error','turn_slip_error')})
    (folder/'cause-qualification.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary), flush=True)
    return 0 if qualified else 1


if __name__ == '__main__':
    raise SystemExit(main())
