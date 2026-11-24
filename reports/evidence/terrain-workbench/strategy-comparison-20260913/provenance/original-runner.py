"""Run a predeclared, reset-per-trial simulation comparison through the workbench API."""
import argparse
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen

from autonomous_survey import STRATEGIES
from survey_evaluation import write_json, write_csv


def request(action='', body=None):
    url='http://127.0.0.1:8766/api/simulation'+('/'+action if action else '')
    req=Request(url,data=json.dumps(body).encode() if body is not None else None,
                headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:8766'})
    with urlopen(req,timeout=15) as response:return json.load(response)


def wait_for(predicate, seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        state=request()
        if predicate(state):return state
        time.sleep(1)
    raise TimeoutError('Workbench state deadline exceeded')


def run(output, budget=90, repeats=2):
    state=request()
    if state.get('motion',{}).get('active') or state.get('autonomy',{}).get('active'):
        raise ValueError('An active movement/task exists; finish it before this comparison')
    output.mkdir(parents=True,exist_ok=False)
    starts=[[3.5,3.,0.],[3.5,-3.,0.]]
    plan=[dict(start=start,repeat=repeat+1,strategy=STRATEGIES[(j+repeat+i)%3],budget_seconds=budget,radius=8)
          for repeat in range(repeats) for i,start in enumerate(starts) for j in range(3)]
    write_json(output/'plan.json',dict(trials=plan,reset='new Gazebo process and empty recorder/SLAM per trial',
               baseline='wait for fresh localization and first archive; record actual initial cells',
               purpose='pilot feasibility comparison; two repeats are not statistical validation'))
    results=[]
    for number,trial in enumerate(plan,1):
        row=dict(trial,index=number,status='starting');results.append(row)
        write_json(output/'trials.json',results)
        print(f'Trial {number}/{len(plan)} {trial}',flush=True)
        try:
            old=(request().get('task') or {}).get('id')
            request('apply',dict(version=1,region=[-25,-25,25,25],start=trial['start']))
            state=wait_for(lambda s:(s.get('task') or {}).get('id') not in (None,old)
                           and s.get('motion',{}).get('ready') and s.get('session'),240)
            ident=state['task']['id'];row['experiment']=ident
            request('explore',dict(experiment=ident,budget_seconds=budget,radius=8,strategy=trial['strategy']))
            state=wait_for(lambda s: bool(s.get('autonomy',{}).get('archive')),20)
            archive=state['autonomy']['archive'];row['archive']=archive
            state=wait_for(lambda s: s.get('autonomy',{}).get('archive')==archive and not s['autonomy']['active'],900)
            row['status']=state['autonomy']['phase'];row['reason']=state['autonomy']['reason']
            folder=Path('/evidence/frontend-experiments')/ident/'research'/Path(archive).stem
            row['folder']=str(folder)
            deadline=time.monotonic()+180
            while time.monotonic()<deadline:
                manifest=folder/'manifest.json'
                if manifest.exists() and json.loads(manifest.read_text()).get('report_status') in ('ready','failed'):break
                time.sleep(1)
            if (folder/'summary.json').exists():row['summary']=json.loads((folder/'summary.json').read_text())
            else:row['report_error']='Report missing at deadline'
        except Exception as error:
            row.update(status='trial_failed',error=str(error))
            request('explore_stop',{})
        write_json(output/'trials.json',results)
        print(json.dumps({k:v for k,v in row.items() if k!='summary'},ensure_ascii=False),flush=True)
    request('stop',{})
    write_csv(output/'comparison.csv',[dict(index=r['index'],start=json.dumps(r['start']),repeat=r['repeat'],
                   **{k:v for k,v in r.items() if k not in ('index','start','repeat','summary')},**r.get('summary',{})) for r in results])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('--budget',type=int,default=90)
    parser.add_argument('--repeats',type=int,default=2)
    args=parser.parse_args();run(args.output,args.budget,args.repeats)
