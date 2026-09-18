"""Frozen endpoint metrics and common cohorts; predictions never use future samples."""
import sys,json,hashlib,math
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from skip_probe import read_prefix
from swept_resampling import snapshot,predict
from resampling_dataset import path_views,path_prediction
from compare_observation_actions import outcome,WINDOW,ARMS
from survey_evaluation import write_json,write_csv,scoped_cells

def evaluate(root,output):
    output.mkdir(parents=True,exist_ok=False);plan=json.loads((root/'plan.json').read_text());trials=[];cohorts={};cells=[];forecasts=[];stages=[]
    for trial in plan['trials']:
        folder=root/trial['folder'];capture=folder/'approach'
        row=dict(folder=trial['folder'],repeat=trial['repeat'],arm=trial['arm'])
        try:
            control=json.loads((folder/'approach-control.json').read_text());row['experiment']=control['experiment']
            assert control['status']=='completed' and control['move_terminal']['phase']=='succeeded','Initial approach/capture failed'
            action=control['comparison'];assert action==json.loads((capture/'comparison.json').read_text())
            assert action['arm']==trial['arm'] and action['goal']==trial['next_goal']
            targets=json.loads((capture/'target-cells.json').read_text());keys={tuple(k) for k in targets['keys']}
            for stem in ('preaction','checkpoint'):
                meta=json.loads((capture/(stem+'.json')).read_text())
                assert hashlib.sha256((capture/(stem+'.npz')).read_bytes()).hexdigest()==meta['sha256']
                with np.load(capture/(stem+'.npz'),allow_pickle=False) as prefix,np.load(capture/'samples.npz',allow_pickle=False) as data:
                    assert set(prefix.files)=={'points','offsets','stamp','estimated_camera','estimated_robot'}
                    for k in prefix.files:assert np.array_equal(prefix[k],data[k][:len(prefix[k])]),'Prefix differs from raw capture'
            initial,_,_,pre=read_prefix(capture,'preaction')
            assert keys=={(c['ix'],c['iy']) for c in initial.quality(True)['cells'] if c['fit_state'] in ('too_few_voxels','insufficient_xy_spread')}
            terrain,camera,robot,meta=read_prefix(capture,'checkpoint');event=action['decision_stamp']
            assert targets==control['frozen_targets'] and pre['sha256']==targets['checkpoint_sha256']
            assert pre['last_stamp']<control['action_started_sim_stamp']<=control['move_terminal']['sim_stamp']<=meta['last_stamp']<=event
            assert 0<=event-meta['last_stamp']<=5 and meta['first_stamp']+trial['seconds']-event>=WINDOW+2
            route=json.loads((capture/'alternative-nav2-plan.json').read_text())
            assert route['experiment']==control['experiment'] and route['query_completed_sim_stamp']<=event
            with np.load(capture/'samples.npz',allow_pickle=False) as data:
                stamps=data['stamp'];assert stamps[-1]>=event+WINDOW,'Incomplete equal window'
                cuts=[int(np.searchsorted(stamps,s,side='right')) for s in (event,event+WINDOW)]
                poses=data['estimated_robot'][cuts[0]-1:cuts[1]]
                distance=float(np.linalg.norm(np.diff(poses[:,:2,3],axis=0),axis=1).sum())
                yaws=np.arctan2(poses[:,1,0],poses[:,0,0]);dy=np.diff(yaws)
                angular=float(np.abs(np.arctan2(np.sin(dy),np.cos(dy))).sum())
                row.update(frames=len(stamps),window_last_sample=float(stamps[cuts[1]-1]),window_boundary_lag=float(event+WINDOW-stamps[cuts[1]-1]))
            before,_,_=snapshot(capture,cuts[0]);after,_,_=snapshot(capture,cuts[1])
            bq,aq=before.quality(True),after.quality(True);origin=trial['start'][:2]
            measured=outcome(bq,aq,keys,origin)
            b=scoped_cells(bq,origin,8);a=scoped_cells(aq,origin,8)
            eligible={k for k,c in b.items() if c['fit_state'] in ('too_few_voxels','insufficient_xy_spread')}
            recovered={k for k in eligible if a.get(k,{}).get('fit_state')=='valid'}
            for k in sorted(b.keys()|a.keys()):cells.append(dict(folder=trial['folder'],ix=k[0],iy=k[1],before=k in b,after=k in a,
                eligible=k in eligible,recovered=k in recovered,target=k in keys))
            terminal=control.get('observe_turn_terminal');move=control.get('compare_move_terminal')
            success=trial['arm']=='hold' and terminal is None and move is None
            actual=control.get('observation_actual_delta')
            if trial['arm'] in ('small_turn','wide_turn'):
                requested=action['requested_delta']
                success=bool(terminal and terminal['phase']=='succeeded' and actual is not None and requested*actual>0 and
                    abs(actual)>=abs(requested)/2 and abs(actual-requested)<=.02+.25*abs(requested))
            if trial['arm']=='advance':
                success=bool(move and move['phase']=='succeeded' and terminal and terminal['phase']=='succeeded' and control.get('observation_reached'))
            complete_stamp=terminal['sim_stamp'] if terminal else event
            if trial['arm']=='advance' and not terminal:complete_stamp=None
            success=success and complete_stamp is not None and complete_stamp<=event+WINDOW
            row.update(status='evaluated',execution_success=success,**measured,estimated_path_m=distance,estimated_abs_yaw_rad=angular,
                action_seconds=complete_stamp-event if complete_stamp is not None else None,actual_delta=actual,
                feature_stamp=meta['last_stamp'],decision_stamp=event,planned_path_m=float(np.linalg.norm(np.diff(np.array(route['path'])[:,:2],axis=0),axis=1).sum()))
            cohorts[trial['folder']]=dict(before=set(b),after=set(a),eligible=eligible,recovered=recovered,targets=keys)
            # Endpoint phases are separate comparisons, never blindly summed after relapses.
            for label,stamp in [('after_move',move.get('sim_stamp') if move else None),('after_turn',terminal.get('sim_stamp') if terminal else None)]:
                if stamp is None or stamp>event+WINDOW:continue
                with np.load(capture/'samples.npz',allow_pickle=False) as data:cut=int(np.searchsorted(data['stamp'],stamp,side='right'))
                stage,_,_=snapshot(capture,cut)
                stages.append(dict(folder=trial['folder'],stage=label,elapsed=stamp-event,**outcome(bq,stage.quality(True),keys,origin)))
            # Predictions use only the checkpoint, exact saved action yaw and queried route.
            quality=terrain.quality(True);quality=dict(quality,cells=list(scoped_cells(quality,origin,8).values()))
            if trial['arm']=='advance':
                views=path_views([p[:2] for p in route['path']],float(camera[2,3]))
                yaw=action['yaw'];views.append((np.r_[np.array(action['goal'])+.435*np.array([math.cos(yaw),math.sin(yaw)]),camera[2,3]],yaw))
                predicted={(c['ix'],c['iy']) for c in quality['cells'] if c['fit_state'] in ('too_few_voxels','insufficient_xy_spread') and path_prediction(c,quality['resolution'],camera[:3,3],views)}
                model='legacy_path_opportunity_not_v4'
            else:
                values,_=predict(quality,terrain.cloud(),camera,robot,action['yaw'],meta['camera_info'],meta['pixel_offset'],WINDOW,gate_version='v3',model_version='v4')
                predicted={(v['ix'],v['iy']) for v in values if v['marginal_prediction']};model='v4_swept_support'
                if trial['arm']=='hold':predicted=set();model='v4_observed_only_hold'
            forecasts.append(dict(folder=trial['folder'],model=model,predicted_recovery_cells=len(predicted),
                tp=len(predicted&recovered),fp=len((predicted&eligible)-recovered),fn=len(recovered-predicted),
                precommand_resolved_or_changed=len(predicted-eligible),predicted_new_area=None,joint_predicted_rank=None))
        except (FileNotFoundError,AssertionError,KeyError,ValueError) as error:
            row.update(status='invalid',error=str(error))
        trials.append(row);print(row['folder'],row['status'],row.get('utility_m2'),flush=True)
    paired=[]
    for repeat in (1,2):
        members={arm:cohorts.get(f'ramp-repeat{repeat}-{arm}') for arm in ARMS}
        if not all(members.values()):continue
        union_before=set.union(*(m['before'] for m in members.values()))
        common=set.intersection(*(m['eligible'] for m in members.values()))
        sharedtargets=set.intersection(*(m['targets'] for m in members.values()))
        base=members['hold'];hold_new=base['after']-union_before;hold_recovered=base['recovered']&common
        for arm,m in members.items():
            new=m['after']-union_before;recovered=m['recovered']&common
            paired.append(dict(repeat=repeat,arm=arm,shared_deficient=len(common),shared_target_deficient=len(common&sharedtargets),
                shared_new_cells=len(new),shared_recovered_cells=len(recovered),shared_target_recovered=len(recovered&sharedtargets),
                new_only_vs_hold=len(new-hold_new),hold_only_new=len(hold_new-new),recovery_only_vs_hold=len(recovered-hold_recovered),
                hold_only_recovery=len(hold_recovered-recovered),net_utility_vs_hold_m2=(len(new)-len(hold_new)+len(recovered)-len(hold_recovered))*.0625))
    complete=len(trials)==8 and all(t['status']=='evaluated' and t['execution_success'] and t['window_boundary_lag']<=1 for t in trials)
    write_csv(output/'trials.csv',trials);write_csv(output/'cells.csv',cells);write_csv(output/'stages.csv',stages);write_csv(output/'forecasts.csv',forecasts)
    write_json(output/'summary.json',dict(trials=trials,paired=paired,forecasts=forecasts,complete=complete,online_ready=False,
        limits=['Two ramp repeats only. Descriptive reset comparisons, not per-cell causal truth or statistical superiority.',
        'Equal20s window includes transit, turn and holding. Sampled pose costs include drift and undersample motion.',
        'Recovery is fitting eligibility, not accuracy; new coverage may include obstacles. Lost valid area is reported separately.',
        'Movement and turn forecasts are different approximations; no comparable total-gain prediction or automatic rank is claimed.']))

if __name__=='__main__':evaluate(Path(sys.argv[1]),Path(sys.argv[2]))
