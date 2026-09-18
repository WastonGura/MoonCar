import sys,json,math,hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from skip_probe import read_prefix,select
from swept_resampling import snapshot
from survey_evaluation import write_json,write_csv

def evaluate(captures,output):
    output.mkdir(parents=True,exist_ok=False);plan=json.loads((captures/'plan.json').read_text());trials=[];rows=[];states={};ids=set();rejections=[]
    for trial in plan['trials']:
        folder=captures/trial['folder'];capture=folder/'approach'
        try:
            control=json.loads((folder/'approach-control.json').read_text());ids.add(control['experiment'])
            assert control['status']=='completed' and control['move_terminal']['phase']=='succeeded'
            targets=json.loads((capture/'target-cells.json').read_text());keys={tuple(k) for k in targets['keys']}
            selection=json.loads((capture/'selection.json').read_text());meta=json.loads((capture/'checkpoint.json').read_text())
            for stem in ('preaction','checkpoint'):
                m=json.loads((capture/(stem+'.json')).read_text())
                assert hashlib.sha256((capture/(stem+'.npz')).read_bytes()).hexdigest()==m['sha256']
                with np.load(capture/(stem+'.npz'),allow_pickle=False) as prefix,np.load(capture/'samples.npz',allow_pickle=False) as data:
                    assert set(prefix.files)=={'points','offsets','stamp','estimated_camera','estimated_robot'}
                    for key in prefix.files:assert np.array_equal(prefix[key],data[key][:len(prefix[key])])
            initial,_,_,premeta=read_prefix(capture,'preaction')
            assert keys=={(c['ix'],c['iy']) for c in initial.quality(True)['cells'] if c['fit_state'] in ('too_few_voxels','insufficient_xy_spread')}
            assert targets==control['frozen_targets'] and targets['checkpoint_sha256']==premeta['sha256']
            assert targets['feature_stamp']<control['action_started_sim_stamp']<=control['move_terminal']['sim_stamp']<=meta['last_stamp']
            event=control['observation_decision_sim_stamp'];assert meta['last_stamp']<=selection['completed_sim_stamp']<=event
            terrain,camera,robot,_=read_prefix(capture,'checkpoint')
            chosen,candidates=select(terrain.quality(True),terrain.cloud(),camera,robot,meta['camera_info'],meta['pixel_offset'],
                meta['first_stamp']+trial['seconds']-meta['last_stamp'],'v4',keys)
            assert chosen==selection['selected'] and candidates==selection['candidates']
            action_ok=False;requested=control.get('observation_requested_delta');actual=control.get('observation_actual_delta')
            if trial['arm']=='turn':
                action_ok=bool(selection['executable'] and chosen and chosen['deficient_cells']>0 and control.get('observation_reached') and
                    requested is not None and actual is not None and abs(requested)>=.01 and requested*actual>0 and
                    abs(actual)>=abs(requested)/2 and abs(actual-requested)<=.02+.25*abs(requested) and event-meta['last_stamp']<=5)
                if not action_ok:rejections.append('negative_action_not_verified:'+trial['folder'])
            else:assert 'observe_turn_terminal' not in control
            with np.load(capture/'samples.npz',allow_pickle=False) as data:
                stamps=data['stamp'];assert stamps[-1]>=event+10
                cuts=[meta['frames'],int(np.searchsorted(stamps,event,side='right')),int(np.searchsorted(stamps,event+10,side='right'))]
            maps=[]
            for cut in cuts:
                t,_,_=snapshot(capture,cut);maps.append({(c['ix'],c['iy']):c['fit_state'] for c in t.quality(True)['cells']})
            deficient={k for k,v in maps[0].items() if v in ('too_few_voxels','insufficient_xy_spread')}
            at_command={k for k in deficient if maps[1].get(k) in ('too_few_voxels','insufficient_xy_spread')}
            recovered={k for k in at_command if maps[2].get(k)=='valid'}
            states[trial['folder']]=dict(targets=keys,eligible=at_command,recovered=recovered)
            for k in sorted(at_command):rows.append(dict(folder=trial['folder'],ix=k[0],iy=k[1],target=k in keys,recovered=k in recovered))
            trials.append(dict(folder=trial['folder'],status='evaluated',experiment=control['experiment'],arm=trial['arm'],
                targets=len(keys),remaining_target_deficient=len(at_command&keys),target_recovered=len(recovered&keys),
                outside_target_recovered=len(recovered-keys),precommand_recovered=sum(maps[1].get(k)=='valid' for k in deficient),
                selected_delta=chosen['delta'] if chosen else None,verified_negative=action_ok,requested=requested,actual=actual,
                turn_seconds=control.get('observe_turn_terminal',{}).get('sim_stamp',event)-event,
                target_stamp=targets['feature_stamp'],action_stamp=control['action_started_sim_stamp'],feature_stamp=meta['last_stamp'],decision_stamp=event))
        except (FileNotFoundError,AssertionError,ValueError,KeyError) as error:
            trials.append(dict(folder=trial['folder'],status='invalid',error=str(error)));rejections.append('invalid:'+trial['folder'])
    pairs=[]
    for pair in sorted({t['pair'] for t in plan['trials']}):
        a,b=states.get(pair+'-turn'),states.get(pair+'-no_turn')
        if a is None or b is None:continue
        common=a['eligible']&b['eligible']
        for scope,cohort in [('shared_targets',common&a['targets']&b['targets']),('all_shared_deficient',common)]:
            extra=cohort&a['recovered']-b['recovered'];hold=cohort&b['recovered']-a['recovered']
            pairs.append(dict(pair=pair,scope=scope,common=len(cohort),turn_only=len(extra),hold_only=len(hold),net=len(extra)-len(hold)))
            if scope=='shared_targets' and (not cohort or extra):rejections.append('empty_target_cohort_or_missed_target_recovery:'+pair)
    if len(ids)!=4 or sum(t.get('verified_negative',False) for t in trials)!=2:rejections.append('insufficient_executed_negative_coverage')
    if sum(p['scope']=='shared_targets' for p in pairs)!=2:rejections.append('insufficient_target_pairs')
    write_csv(output/'cells.csv',rows)
    write_json(output/'summary.json',dict(trials=trials,pairs=pairs,rejections=rejections,target_screen_passed=not rejections,
        online_ready=False,scope='Frozen pre-move deficient targets only',
        limits=['Two ramp repeats, exploratory screen only. Separate resets/target sets/poses limit causal conclusions.',
        'Outside-target gains are retained; skipping this subtask cannot be equated with no global information benefit.',
        'No learned model, optimality proof, new full three-strategy comparison, or hardware validation.']))
    print(json.dumps(dict(trials=trials,pairs=pairs,rejections=rejections),indent=2),flush=True)

if __name__=='__main__':evaluate(Path(sys.argv[1]),Path(sys.argv[2]))
