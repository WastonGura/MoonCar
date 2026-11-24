"""Check compact offline readers and timestamped on-disk map retrieval."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
LEGACY=ROOT/'reports/v3_mapping_comparison_20261117'
sys.path.insert(0,str(LEGACY))


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


def main():
    video=module('compact_video',ROOT/'reports/turn_minimap_20261111/render_video.py')
    compose=module('indexed_maps',LEGACY/'compose.py')
    duration=module('duration_analysis',Path(__file__).with_name('analyze_duration.py'))
    with tempfile.TemporaryDirectory() as temporary:
        folder=Path(temporary)
        state=dict(wall=2.,state=dict(motion=dict(pose={'x':1.,'y':2.},goal={'x':3.},diagnostics={'sim_stamp':1.5,'unused':42}),autonomy=dict(phase='moving',reason='observing',history=[{'unused':1}]*100)))
        path=folder/'states.jsonl';path.write_text(json.dumps(state)+'\n',encoding='utf-8')
        compact=video.state_rows(path)[0]
        assert compact['wall']==state['wall']
        for key in ('pose','goal'):assert compact['state']['motion'][key]==state['state']['motion'][key]
        assert compact['state']['motion']['diagnostics']['sim_stamp']==1.5
        assert compact['state']['autonomy']==dict(phase='moving',reason='observing')
        state['state']['autonomy']={}
        path.write_text(json.dumps(state)+'\n',encoding='utf-8')
        assert video.state_rows(path)[0]['state']['autonomy']=={}
        path=folder/'samples.jsonl'
        command=dict(topic='/mooncar/cmd_vel_safe',sim=2.,linear=.1,angular=.2)
        path.write_text('\n'.join(json.dumps(r) for r in ({'topic':'unrelated'},command))+'\n',encoding='utf-8')
        assert video.safe_commands(path)==[command]
        path=folder/'survey-map.jsonl';headers=[]
        with path.open('wb') as stream:
            for i in range(2):
                row=dict(wall=10.+i,sim=2.+i,payload=dict(stamp=1.+i,frames=i+1,resolution=.25,cells=[dict(ix=i,iy=-3,z=.123+i,valid=True)]))
                headers.append(dict(wall=row['wall'],sim=row['sim'],offset=stream.tell(),payload={k:row['payload'][k] for k in ('stamp','frames')}))
                stream.write((json.dumps(row)+'\n').encode())
        source=dict(map_path=path,snapshots=headers,map_walls=[r['wall'] for r in headers])
        assert compose.map_payload(source,0)['cells'][0]['z']==.123
        assert compose.map_payload(source,-1)['cells'][0]['z']==1.123
        assert compose.select_map(source,10.5,2.5)==0
        assert compose.select_map(source,12.,2.5)==0
        assert compose.select_map(source,9.,2.5)==-1
        case=folder/'baseline/case';observer=case/'observer';observer.mkdir(parents=True)
        times=(0.,30.,60.,90.,240.,480.)
        def jsonl(path,values):path.write_text(''.join(json.dumps(v)+'\n' for v in values),encoding='utf-8')
        jsonl(case/'states.jsonl',[dict(wall=t+1,state=dict(motion=dict(diagnostics={'sim_stamp':t}),autonomy={'phase':'moving'})) for t in times])
        jsonl(observer/'survey-map.jsonl',[dict(wall=t+1,sim=t,payload=dict(stamp=t,frames=i+1,resolution=.25,
            cells=[dict(ix=j,iy=0,z=0.,slope=0.,valid=True,fit_state='valid') for j in range(i+1)])) for i,t in enumerate(times)])
        jsonl(observer/'truth.jsonl',[dict(stamp=t,xyz=[i*.1,0.,0.]) for i,t in enumerate(times)])
        jsonl(observer/'alignment.jsonl',[dict(stamp=t,xy_error_m=.001*i) for i,t in enumerate(times)])
        jsonl(observer/'samples.jsonl',[dict(topic='/mooncar/map/status',sim=90.,wall=91.,status=[dict(level=1,message='Session capacity reached; save and start another session')])])
        jsonl(case/'resources.jsonl',[dict(sim=t,localization_rss_kib=128000+i*1024) for i,t in enumerate(times)])
        reference=dict(z=np.zeros((120,120)),slope=np.zeros((120,120)),mixed=np.zeros((120,120),bool))
        def analyze(elapsed,count):
            mission=dict(started=0.,elapsed_seconds=elapsed,budget_seconds=480.,wall_limit_seconds=7200.,origin=[0.,0.],
                         history=[dict(kind='advance',started_seconds=0.,success=True)])
            (case/'result.json').write_text(json.dumps(dict(autonomy=mission)),encoding='utf-8')
            summary=dict(terminal_receipt_wall=elapsed+1,snapshot_receipt_wall=elapsed+1,stamp=elapsed,
                         total_observed_m2=count*.25**2,outcome='failed' if elapsed<480 else 'budget_completed',reason='test')
            return duration.analyze(folder,'baseline',summary,reference,30.)
        stopped=analyze(90.,4)
        assert len(stopped['minute_metrics'])==3 and not stopped['half_windows'][1]['reached']
        assert stopped['common_duration_metrics']['total_observed_m2']==2*.25**2
        assert stopped['first_capacity_warning']['elapsed_seconds']==90.
        complete=analyze(480.,6)
        assert complete['half_windows'][1]['new_observed_m2']==.25**2
        assert complete['minute_metrics'][-1]['total_observed_m2']==6*.25**2
    receipt=dict(passed=True,checks=['compact_fields_preserved','command_values_preserved','exact_payload_byte_offset','no_future_receipt','no_future_sim','unknown_before_first_map','early_stop_not_extended','common_duration_uses_prefix','second_half_gain_from_real_boundary'])
    Path(__file__).with_name('offline-tool-checks.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(receipt))


if __name__=='__main__':main()
