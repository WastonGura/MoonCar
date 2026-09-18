"""Descriptive, paired shadow validation of the predeclared arrival gate."""
import csv,json,shutil
from pathlib import Path
from survey_evaluation import write_json,write_csv

ELIGIBLE={'too_few_voxels','insufficient_xy_spread'}


def paired(a,b,target=None):
    def select(rows):
        return {(r['ix'],r['iy']):r for r in rows if r['decision'] in ELIGIBLE
                and (target is None or r['baseline_surface']==target)}
    a,b=select(a),select(b);keys=a.keys()&b.keys()
    known={k for k in keys if a[k]['final']!='missing' and b[k]['final']!='missing'}
    na=sum(a[k]['final']=='valid' for k in known);nb=sum(b[k]['final']=='valid' for k in known)
    return dict(common_remaining=len(keys),missing_pairs=len(keys-known),no_turn_recovered=na,
                turn_recovered=nb,net_recovered=nb-na,
                turn_only=sum(a[k]['final']!='valid' and b[k]['final']=='valid' for k in known),
                no_turn_only=sum(a[k]['final']=='valid' and b[k]['final']!='valid' for k in known))


def report(bundle):
    plan=json.loads((bundle/'captures/plan.json').read_text())
    stages=json.loads((bundle/'stages/summary.json').read_text())
    with (bundle/'stages/cells.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
    trials=[];pairs=[];rejections=[]
    for t in stages['trials']:
        if t['status']!='evaluated':rejections.append(dict(folder=t['folder'],reason='execution_failure'));continue
        decision=t['decisions']['focus'];selected=[r for r in rows if r['folder']==t['folder'] and r['decision'] in ELIGIBLE]
        assert len(selected)==decision['remaining_deficient']
        missed=sum(r['final']=='valid' for r in selected if [int(r['ix']),int(r['iy'])] not in decision['opportunity_cells'])
        arm=next(p['arm'] for p in plan['trials'] if p['folder']==t['folder'])
        control=json.loads((bundle/'captures'/t['folder']/'approach-control.json').read_text())
        observation_ok=arm=='no_turn' or control.get('observation_reached') is True
        trials.append(dict(folder=t['folder'],arm=arm,**decision,post_decision_recovered=sum(r['final']=='valid' for r in selected),
                           missed_recovery_cells=missed,observation_executed_ok=observation_ok,
                           decision_data_age=t['decision_event_stamp']-t['stage_last_stamps']['decision']))
        if not observation_ok:rejections.append(dict(folder=t['folder'],reason='observation_heading_not_reached'))
        if t['stage_last_stamps']['decision']<t['arrival_event_stamp']:
            rejections.append(dict(folder=t['folder'],reason='no_post_arrival_observation'))
        if decision['reason']=='no_predicted_remaining_recovery' and missed:
            rejections.append(dict(folder=t['folder'],reason='zero_opportunity_but_later_recovery',cells=missed))
    for pair in sorted({p['pair'] for p in plan['trials']}):
        a,b=[next(p for p in plan['trials'] if p['pair']==pair and p['arm']==arm) for arm in ('no_turn','turn')]
        if not all(any(t['folder']==p['folder'] for t in trials) for p in (a,b)):continue
        ra=[r for r in rows if r['folder']==a['folder']];rb=[r for r in rows if r['folder']==b['folder']]
        for scope,target in [('all_common_remaining',None),('pure_target','sloped_top' if 'ramp' in a['scene'] else 'curved_top')]:
            values=paired(ra,rb,target);pairs.append(dict(pair=pair,scope=scope,**values))
            if scope=='all_common_remaining' and not next(t for t in trials if t['folder']==b['folder'])['turn'] and values['turn_only']:
                rejections.append(dict(pair=pair,reason='skip_would_miss_paired_turn_only_recovery',cells=values['turn_only']))
    coverage=dict(try_turn=sum(t['turn'] for t in trials),skip_turn=sum(not t['turn'] for t in trials))
    if not all(coverage.values()):rejections.append(dict(reason='both_decision_branches_not_validated',**coverage))
    if len(trials)!=len(plan['trials']):rejections.append(dict(reason='incomplete_successful_validation'))
    write_csv(bundle/'stages/gate-trials.csv',trials);write_csv(bundle/'stages/gate-pairs.csv',pairs)
    write_json(bundle/'stages/gate-assessment.json',dict(trials=trials,pairs=pairs,rejections=rejections,
        shadow_check_passed=not rejections,decision_coverage=coverage,online_policy_changed=False,
        limits=['Both arms are actually executed independently of the gate; this is shadow validation, not a deployed adaptive experiment.',
        'Post-decision recovery can also occur while holding; it contradicts a zero-recovery prediction but does not prove a causal turn benefit.',
        'Common decision-time cohorts can differ from common initial cohorts. Geometry, ranges and two repeats remain limited.']))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected=[p for p in pairs if p['scope']=='all_common_remaining']
    fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
    for offset,key,label,hatch in [(-.18,'no_turn_recovered','Hold heading',''),(.18,'turn_recovered','Turn heading','//')]:
        for i,p in enumerate(selected):
            ax.bar(i+offset,p[key],.36,color='white',edgecolor='black',hatch=hatch,label=label if i==0 else None)
            ax.text(i+offset,p[key]+.1,str(p[key]),ha='center',fontsize=9)
    ax.set_xticks(range(len(selected)),[p['pair'].replace('visible_','').replace('_v1','')+f"\nshared n={p['common_remaining']}" for p in selected],fontsize=9)
    ax.set_ylabel('Recovered shared deficient cells');ax.set_title('Recovery after the arrival decision');ax.legend()
    fig.supxlabel('Independent captures, small repeated sample; cell counts are not accuracy or independent trials.',fontsize=8)
    fig.savefig(bundle/'stages/paired-recovery.png',dpi=180);plt.close(fig)
    shutil.copyfile(Path(__file__),bundle/'stages'/Path(__file__).name)
    print(json.dumps(dict(trials=trials,pairs=pairs,rejections=rejections),indent=2))


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path)
    report(p.parse_args().bundle)
