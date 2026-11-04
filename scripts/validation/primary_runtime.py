from pathlib import Path
import os
import sys
sys.path.insert(0,'/source/scripts')
from rgbd_shadow_runtime import owned_process,threads,set_threads

def check(record):
    for key in ('localization','gazebo'):
        item=record[key]
        if any(not set(t['cpus']).issubset(item['cpus']) for t in threads(item['process'])):
            raise RuntimeError('Primary resource profile changed: '+key)
    if any(t['nice']!=0 for t in threads(record['localization']['process'])):raise RuntimeError('Primary priority changed')

def apply(experiment,receipt):
    primary=owned_process(experiment,lambda a:any(Path(x).name=='rgbd_window_odometry.py' for x in a))
    if any(not set(t['cpus']).issubset({0,1}) for t in threads(primary)):raise RuntimeError('Primary startup affinity missing')
    try:
        old=owned_process(experiment,lambda a:any(Path(x).name=='rgbd_odometry_node.py' for x in a))
    except RuntimeError as error:
        if '[]' not in str(error):raise
    else:raise RuntimeError('Unexpected simultaneous legacy producer: '+str(old))
    gazebo=owned_process(experiment,lambda a:(len(a)==1 and a[0].startswith('gz sim ')) or (len(a)>1 and Path(a[0]).name=='gz' and a[1]=='sim'))
    g=set_threads(gazebo,{2,3,4,5})
    record=dict(localization=dict(process=primary,cpus=[0,1]),gazebo=g,authority='rgbd_window',legacy_producer_absent=True)
    import json
    Path(receipt).write_text(json.dumps(record,indent=2)+'\n');check(record);return record
