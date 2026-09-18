"""Run a predeclared, reset-per-trial simulation comparison through the workbench API."""
import argparse
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from autonomous_survey import STRATEGIES
from survey_evaluation import write_json, write_csv


def read_pending_json(path):
    """A polling reader can race an atomic replacement on the Windows bind mount."""
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (FileNotFoundError,json.JSONDecodeError):return None


def request(action='', body=None):
    url='http://127.0.0.1:8766/api/simulation'+('/'+action if action else '')
    req=Request(url,data=json.dumps(body).encode() if body is not None else None,
                headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:8766'})
    try:
        with urlopen(req,timeout=15) as response:return json.load(response)
    except HTTPError as error:
        raise ValueError(f'HTTP {error.code}: {error.read(2048).decode("utf-8",errors="replace")}') from error


def wait_for(predicate, seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        state=request()
        if predicate(state):return state
        time.sleep(1)
    raise TimeoutError('Workbench state deadline exceeded')


def run(output, budget=90, repeats=2, starts=None, strategies=STRATEGIES):
    if not 15<=budget<=1800 or not 1<=repeats<=10:raise ValueError('Budget 15..1800 and repeats 1..10 required')
    if not strategies or len(set(strategies))!=len(strategies) or any(s not in STRATEGIES for s in strategies):
        raise ValueError('Choose distinct supported strategies')
    state=request()
    if state.get('motion',{}).get('active') or state.get('autonomy',{}).get('active'):
        raise ValueError('An active movement/task exists; finish it before this comparison')
    starts=starts or [[3.5,3.,0.],[0.,-3.,0.]]
    from simulation_experiment import validate_task
    for start in starts:validate_task(dict(version=1,region=[-25,-25,25,25],start=start))
    output.mkdir(parents=True,exist_ok=False)
    plan=[dict(start=start,repeat=repeat+1,strategy=strategies[(j+repeat+i)%len(strategies)],budget_seconds=budget,radius=8)
          for repeat in range(repeats) for i,start in enumerate(starts) for j in range(len(strategies))]
    write_json(output/'plan.json',dict(trials=plan,reset='new Gazebo process and empty recorder/SLAM per trial',
               baseline='wait for fresh localization and first archive; record actual initial cells',
               purpose='pilot feasibility comparison; two repeats are not statistical validation'))
    results=[]
    for number,trial in enumerate(plan,1):
        row=dict(trial,index=number,status='starting');results.append(row)
        write_json(output/'trials.json',results)
        print(f'Trial {number}/{len(plan)} {trial}',flush=True)
        old=None;applied=False
        try:
            old=(request().get('task') or {}).get('id')
            request('apply',dict(version=1,region=[-25,-25,25,25],start=trial['start']))
            applied=True
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
                if (read_pending_json(manifest) or {}).get('report_status') in ('ready','failed'):break
                time.sleep(1)
            if (folder/'summary.json').exists():row['summary']=json.loads((folder/'summary.json').read_text())
            else:row['report_error']='Report missing at deadline'
        except Exception as error:
            row.update(status='trial_failed',error=str(error))
            try:
                state=request();task=state.get('task') or {}
                if applied and task.get('id') not in (None,old):
                    row['experiment']=task['id']
                    write_json(output/f'trial-{number:02d}-failure-state.json',state)
                request('explore_stop',{})
            except Exception as diagnostic_error:
                row['diagnostic_error']=str(diagnostic_error);write_json(output/'trials.json',results)
                raise
        write_json(output/'trials.json',results)
        print(json.dumps({k:v for k,v in row.items() if k!='summary'},ensure_ascii=False),flush=True)
    request('stop',{})
    write_csv(output/'comparison.csv',[{**{k:v for k,v in r.items() if k!='summary'},
                   **r.get('summary',{}),'start':json.dumps(r['start'])} for r in results])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('--budget',type=int,default=90)
    parser.add_argument('--repeats',type=int,default=2)
    parser.add_argument('--start',type=float,nargs=3,action='append')
    parser.add_argument('--strategy',choices=STRATEGIES,action='append')
    args=parser.parse_args();run(args.output,args.budget,args.repeats,args.start,args.strategy or STRATEGIES)
