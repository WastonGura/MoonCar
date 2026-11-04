"""Reference-based single-surface scoring. Never imported by online planning."""
import math
import numpy as np
from terrain_reference import reference_grid

VERSION='mapping-acceptance-v1.2'
PROFILE=dict(height_tolerance_m=.05, slope_tolerance_deg=5.,
             minimum_precision=.9, status='provisional_simulation_targets')


def validate_profile(profile):
    for name in ('height_tolerance_m','slope_tolerance_deg','minimum_precision'):
        value=profile.get(name)
        if type(value) not in (float,int) or not math.isfinite(value) or value<=0:
            raise ValueError(f'Invalid profile field: {name}')
    if profile['slope_tolerance_deg']>90 or profile['minimum_precision']>1:
        raise ValueError('Invalid degree or fraction thresholds')
    # Accept archived profiles, but never use their former coverage threshold.
    if 'minimum_coverage' in profile:
        value=profile['minimum_coverage']
        if type(value) not in (float,int) or not math.isfinite(value) or not 0<value<=1:
            raise ValueError('Invalid legacy profile field: minimum_coverage')


def checked_cells(cells, reference=False):
    result={}
    for cell in cells:
        if any(type(cell.get(k)) is not int for k in ('ix','iy')):
            raise ValueError('Integer cell coordinates required')
        key=(cell['ix'],cell['iy'])
        if key in result:raise ValueError('Duplicate cell coordinates')
        if reference:
            if cell.get('category') not in ('ground','terrain','edge_or_complex','no_reference'):
                raise ValueError('Unknown reference category')
            required=cell['category'] in ('ground','terrain')
        else:
            if type(cell.get('valid')) is not bool:raise ValueError('Boolean valid field required')
            required=cell['valid']
        if required:
            if any(type(cell.get(k)) not in (float,int) or not math.isfinite(cell[k]) for k in ('z','slope')):
                raise ValueError('Finite height and slope required')
            if not 0<=cell['slope']<=90:raise ValueError('Slope outside 0..90 degrees')
        if any(cell.get(k) is not None and (type(cell[k]) not in (float,int) or not math.isfinite(cell[k])) for k in ('z','slope')):
            raise ValueError('Optional height and slope must be finite or null')
        if reference and any(type(cell.get(k)) not in (float,int) or not math.isfinite(cell[k]) for k in ('x','y')):
            raise ValueError('Finite reference cell centres required')
        result[key]=cell
    return result


def build_reference(shapes,origin,radius,resolution=.25):
    """Reference-only mask, fixed before examining any rover output."""
    coarse=reference_grid(shapes,origin,radius,resolution,17)
    fine=reference_grid(shapes,origin,radius,resolution,33)
    assert [(c['ix'],c['iy']) for c in coarse]==[(c['ix'],c['iy']) for c in fine]
    excluded_unstable=[];mask_disagreement=[];missing=[]
    for a,b in zip(coarse,fine):
        key=[b['ix'],b['iy']]
        if b['category']=='no_reference' or a['category']=='no_reference':missing.append(key)
        if a['category']!=b['category']:mask_disagreement.append(key)
        if a['category'] in ('ground','terrain') and b['category'] in ('ground','terrain'):
            if abs(a['z']-b['z'])>.001 or abs(a['slope']-b['slope'])>.1:
                b['category']='edge_or_complex';excluded_unstable.append(key)
    audit=dict(method='highest analytic visual surface; 17x17 vs33x33 central-cell samples; nonconvergent cells excluded',
               height_convergence_m=.001,slope_convergence_deg=.1,
               unstable_cells=[],excluded_unstable_cells=excluded_unstable,
               mask_disagreement=mask_disagreement,missing_reference=missing,
               passed=not(mask_disagreement or missing),
               limitation='Nonconvergent cells are excluded from the fixed denominator; convergence is numerical evidence, not proof of mesh fidelity or sub-sample feature detection.')
    return dict(version=VERSION,origin=list(origin),radius=radius,resolution=resolution,
                cells=fine,reference_audit=audit)


def score(reference,estimates,*,evidence,profile=None):
    """Score all declared ROI cells; missing/wrong estimates never shrink denominators."""
    profile=dict(PROFILE if profile is None else profile);validate_profile(profile)
    profile.pop('minimum_coverage',None)
    r=reference.get('resolution')
    if type(r) not in (float,int) or not math.isfinite(r) or r<=0:raise ValueError('Invalid grid resolution')
    refs=checked_cells(reference['cells'],True);est=checked_cells(estimates)
    if any(not math.isclose(c[k],(c[i]+.5)*r,abs_tol=1e-9,rel_tol=0)
           for c in refs.values() for k,i in (('x','ix'),('y','iy'))):
        raise ValueError('Reference centres do not match declared grid')
    problems=list(evidence.get('problems',[]))
    if not reference.get('reference_audit',{}).get('passed'):problems.append('Reference audit not passed')
    for key in ('coordinate_contract_verified','sensor_lineage_verified','cutoff_verified'):
        if evidence.get(key) is not True:problems.append(key+' not verified')
    rows=[]
    for key,ref in refs.items():
        eligible=ref['category'] in ('ground','terrain');cell=est.get(key)
        observed=cell is not None;valid=observed and cell['valid']
        ez=cell['z']-ref['z'] if eligible and valid else None
        es=cell['slope']-ref['slope'] if eligible and valid else None
        good=ez is not None and abs(ez)<=profile['height_tolerance_m'] and abs(es)<=profile['slope_tolerance_deg']
        state=('outside_representation' if not eligible else 'unobserved' if not observed else
               'invalid_fit' if not valid else 'qualified' if good else 'error_exceeds_threshold')
        rows.append(dict(ix=key[0],iy=key[1],x=ref['x'],y=ref['y'],category=ref['category'],eligible=eligible,
            observed=observed,valid=bool(valid),qualified=good,state=state,
            reference_z=ref['z'],reference_slope=ref['slope'],estimated_z=cell.get('z') if cell else None,
            estimated_slope=cell.get('slope') if cell else None,height_error_m=ez,slope_error_deg=es))
    if any(c['category']=='no_reference' for c in rows):problems.append('Reference missing within declared task ROI')
    if not any(c['eligible'] for c in rows):problems.append('Empty eligible ROI')
    evaluable=not problems
    def aggregate(selected):
        eligible=[c for c in selected if c['eligible']];observed=[c for c in eligible if c['observed']]
        valid=[c for c in eligible if c['valid']];good=[c for c in eligible if c['qualified']]
        n=len(eligible);a=r*r
        height=[c['height_error_m'] for c in valid];slope=[c['slope_error_deg'] for c in valid]
        coverage=len(valid)/n if n else None;precision=len(good)/len(valid) if valid else None
        return dict(task_cells=len(selected),eligible_cells=n,excluded_cells=len(selected)-n,
            task_area_m2=len(selected)*a,evaluation_area_m2=n*a,representation_fraction=n/len(selected) if selected else None,
            excluded_area_m2=(len(selected)-n)*a,
            observed_cells=len(observed),estimated_cells=len(valid),qualified_cells=len(good) if evaluable else None,
            C_obs=len(observed)/n if n and evaluable else None,C=coverage if evaluable else None,
            P=precision if evaluable else None,S=100*len(good)/n if n and evaluable else None,
            qualified_area_m2=len(good)*a if evaluable else None,
            height_rmse_m=math.sqrt(sum(e*e for e in height)/len(height)) if height and evaluable else None,
            height_p95_abs_m=float(np.percentile(np.abs(height),95)) if height and evaluable else None,
            slope_mae_deg=sum(abs(e) for e in slope)/len(slope) if slope and evaluable else None,
            geometry_pass=bool(evaluable and n and
                               precision is not None and precision>=profile['minimum_precision']))
    summary=aggregate(rows)
    summary['by_category']={name:aggregate([c for c in rows if c['category']==name]) for name in ('ground','terrain')}
    # Unmeasured categories have unknown accuracy, not a hidden coverage gate.
    unassessed=[name for name,v in summary['by_category'].items() if v['eligible_cells'] and not v['estimated_cells']]
    geometry_pass=summary['geometry_pass'] and all(v['geometry_pass'] for v in summary['by_category'].values() if v['estimated_cells'])
    failed_gates=[]
    if evaluable:
        for name,value in [('all',summary),*summary['by_category'].items()]:
            if not value['eligible_cells'] or (name!='all' and not value['estimated_cells']):continue
            if value['P'] is None or value['P']<profile['minimum_precision']:failed_gates.append(name+': precision below target or undefined')
        if evidence.get('runtime_fault') is True:failed_gates.append('Recorded execution fault')
        if evidence.get('safety_pass') is False:failed_gates.append('Recorded safety failure')
    if not evaluable:status='not_evaluable'
    elif not geometry_pass or evidence.get('safety_pass') is False or evidence.get('runtime_fault') is True:status='failed'
    elif evidence.get('safety_pass') is not True:status='geometry_pass_safety_unverified'
    else:status='passed_under_simulation_profile'
    # No apparent per-cell pass colours or precision are published when reference/lineage is unverified.
    if not evaluable:
        for row in rows:
            row.update(qualified=None,height_error_m=None,slope_error_deg=None)
            if row['eligible']:row['state']='not_evaluable'
    summary.update(version=VERSION,status=status,evaluable=evaluable,geometry_pass=geometry_pass,
                   coverage_gate_enabled=False,coverage_role='descriptive_only',unassessed_categories=unassessed,
                   profile=profile,evidence=evidence,problems=problems,failed_gates=failed_gates,
                   outside_roi_estimate_cells=len(est.keys()-refs.keys()))
    return rows,summary
