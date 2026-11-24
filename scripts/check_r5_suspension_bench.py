"""Real native-DART single-joint load, decay and travel-limit tests."""
import argparse,hashlib,json,os,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ENGINE='/opt/mooncar/physics/dart-7.8.0/libmooncar-physics7-dartsim.so'
OBSERVER='/work/suspension-observer/libmooncar-suspension-observer.so'

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False);results=[]
    for name,g,ref,target in [('load',-1.62,0,-.01),('decay',0,.01,.01),('limit',-16.2,0,-.02)]:
        root=ET.Element('sdf',version='1.10');world=ET.SubElement(root,'world',name='bench')
        plugin=ET.SubElement(world,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics')
        ET.SubElement(ET.SubElement(plugin,'engine'),'filename').text=ENGINE
        physics=ET.SubElement(world,'physics',name='physics',type='ode');ET.SubElement(physics,'max_step_size').text='.001';ET.SubElement(physics,'real_time_factor').text='0'
        ET.SubElement(world,'gravity').text=f'0 0 {g}'
        model=ET.SubElement(world,'model',name='bench')
        for n,m in [('anchor',1),('load',2)]:
            link=ET.SubElement(model,'link',name=n);ET.SubElement(link,'pose').text='0 0 1 0 0 0'
            i=ET.SubElement(link,'inertial');ET.SubElement(i,'mass').text=str(m);tensor=ET.SubElement(i,'inertia')
            for k in ('ixx','iyy','izz','ixy','ixz','iyz'):ET.SubElement(tensor,k).text='.01' if k in ('ixx','iyy','izz') else '0'
        j=ET.SubElement(model,'joint',name='anchor_fixed_joint',type='fixed');ET.SubElement(j,'parent').text='world';ET.SubElement(j,'child').text='anchor'
        j=ET.SubElement(model,'joint',name='spring',type='prismatic');ET.SubElement(j,'parent').text='anchor';ET.SubElement(j,'child').text='load'
        axis=ET.SubElement(j,'axis');ET.SubElement(axis,'xyz').text='0 0 1';d=ET.SubElement(axis,'dynamics')
        for k,v in [('spring_stiffness',324),('spring_reference',ref),('damping',2*.7*np.sqrt(648))]:ET.SubElement(d,k).text=str(v)
        lim=ET.SubElement(axis,'limit');ET.SubElement(lim,'lower').text='-.02';ET.SubElement(lim,'upper').text='.02'
        csv=out/(name+'.csv');plugin=ET.SubElement(model,'plugin',filename=OBSERVER,name='mooncar::SuspensionObserver');ET.SubElement(plugin,'output').text=str(csv);ET.SubElement(plugin,'joint').text='spring'
        path=out/(name+'.sdf');ET.ElementTree(root).write(path)
        env=dict(os.environ);env['LD_LIBRARY_PATH']='/opt/mooncar/turn/r5-v2:'+env.get('LD_LIBRARY_PATH','')
        with (out/(name+'.log')).open('w') as log:
            proc=subprocess.Popen(['gz','sim','-s','-r','--iterations','6000',str(path)],stdout=log,stderr=subprocess.STDOUT,env=env)
            loaded=[];deadline=time.monotonic()+60
            while proc.poll() is None and time.monotonic()<deadline:
                for f in Path('/proc').iterdir():
                    if not f.name.isdigit():continue
                    try:
                        cmd=(f/'cmdline').read_bytes()
                        if str(path).encode() not in cmd:continue
                        matches=[l.split()[-1] for l in (f/'maps').read_text().splitlines() if ENGINE in l]
                        if matches:loaded=sorted(set(matches))
                    except (FileNotFoundError,ProcessLookupError):pass
                time.sleep(.02)
            if proc.poll() is None:proc.terminate();proc.wait(timeout=10);raise TimeoutError(name)
        data=np.genfromtxt(csv,delimiter=',',names=True);q=data['spring_q'];v=data['spring_v']
        error=abs(float(q[-1000:].mean())-target)
        envelope=[float(np.max(abs(q[(data['t']>=i)&(data['t']<i+1)]-target))) for i in range(6)]
        passed=bool(proc.returncode==0 and loaded==[ENGINE] and len(q)==6000 and np.isfinite(q).all() and
            error<=.0005 and np.max(abs(q))<=.02001 and np.max(abs(v[-1000:]))<.001 and
            (name!='decay' or envelope[-1]<envelope[0]*.01))
        row=dict(name=name,passed=passed,expected_q=target,final_q=float(q[-1]),error_m=error,envelope_m=envelope,
            steps=len(q),loaded=loaded,engine_sha256=hashlib.sha256(Path(ENGINE).read_bytes()).hexdigest())
        results.append(row);(out/'result.json').write_text(json.dumps(dict(passed=all(r['passed'] for r in results),results=results),indent=2)+'\n')
        print(json.dumps(row),flush=True)
        if not passed:return 1
    return 0
if __name__=='__main__':raise SystemExit(main())
