"""Offline replay of actual preparation hooks, without importing ROS or Gazebo."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent
EVIDENCE=ROOT.parent
WORKSPACE=ROOT.parents[3]
PREP=EVIDENCE/'r5-v2-offset-diagnosis-prep-20261011'
SCRIPTS=WORKSPACE/'scripts'
BASE=EVIDENCE/'r5-turn-gate-validation-20261011/campaign/14-pure_turn-ramp-1000us-left'


def source_tree(path):return ast.parse(path.read_text(encoding='utf-8'))


def extracted_prepare(path,context):
    functions=[n for n in ast.walk(source_tree(path)) if isinstance(n,ast.FunctionDef) and n.name=='prepare']
    assert len(functions)==1
    namespace=dict(context)
    exec(compile(ast.Module(body=[functions[0]],type_ignores=[]),str(path),'exec'),namespace)
    return namespace['prepare']


def state(path):
    model=ET.parse(path).find("world/model[@name='mooncar']")
    return {'pose':model.findtext('pose'),
            'contact_observer_count':len(model.findall("plugin[@name='mooncar::ContactObserver']")),
            'step_observer_count':len(model.findall("plugin[@name='mooncar::StepObserver']"))}


def qualification(r,code):
    function=next(n for n in source_tree(PREP/'check_r5_v2_offset_round.py').body if isinstance(n,ast.FunctionDef) and n.name=='main')
    statements=[];capture=False
    for node in function.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='probe' for t in node.targets):capture=True
        if capture and isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=='write_text':break
        if capture:statements.append(node)
    context={'r':r,'code':code,'args':SimpleNamespace(name='offline-test'),
             'world_src':PREP/'worlds/world_shift_x05.sdf',
             'SCREEN_ERROR':"ValueError('Existing 1 cm turn screen failed')"}
    exec(compile(ast.Module(body=statements,type_ignores=[]),'actual_qualification_block','exec'),context)
    return context['diag_result']


def main():
    # The only substitute is the lowest-level original preparation's file setup.
    # All three wrapping functions and runtime observer injection are actual AST.
    replay=ROOT/'prepare-replay';replay.mkdir(exist_ok=True)
    def base_prepare(args,out):
        path=out/'world.sdf';shutil.copy2(BASE/'world.sdf',path);return path
    offset=extracted_prepare(PREP/'check_r5_v2_offset_round.py',{
        'original_prepare':base_prepare,'world_src':PREP/'worlds/world_shift_x05.sdf','shutil':shutil})
    gate=extracted_prepare(SCRIPTS/'check_r5_turn_slip_round.py',{
        'old_prepare':offset,'ET':ET,'controller':Path('/work/test_shift_x_plus_5cm-axle-commands.csv'),
        'gate':Path('/work/test_shift_x_plus_5cm-slip-control.csv'),'contact':Path('/work/test_shift_x_plus_5cm-contact-steps.csv')})
    origin=extracted_prepare(SCRIPTS/'check_r5_origin_round.py',{
        'original_prepare':gate,'ET':ET,'a':SimpleNamespace(spawn_yaw=0.)})
    result={'scope':'Offline AST hook replay only; no ROS imports, Docker, Gazebo or physical trials',
            'declared_shift_world':state(PREP/'worlds/world_shift_x05.sdf')}
    path=origin(SimpleNamespace(),replay)
    result['after_actual_prepare_wrappers']=state(path)
    tree=ET.parse(path);world=tree.find('world')
    runtime_run=next(n for n in source_tree(SCRIPTS/'check_r5_runtime.py').body if isinstance(n,ast.FunctionDef) and n.name=='run')
    injection=next(n for n in runtime_run.body if isinstance(n,ast.If) and ast.unparse(n.test)=="getattr(args, 'observer_plugin', None)")
    namespace={'args':SimpleNamespace(observer_plugin='/evidence/observer/libmooncar-origin-observer.so',name='test_shift_x_plus_5cm'),
               'ET':ET,'Path':Path,'w':world}
    exec(compile(ast.Module(body=[injection],type_ignores=[]),'actual_runtime_observer_injection','exec'),namespace)
    tree.write(path)
    result['after_runtime_observer_injection']=state(path)
    raw=json.loads((BASE/'result.json').read_text())
    result['qualification_from_old_round']=qualification(copy.deepcopy(raw),1)
    invalid=copy.deepcopy(raw);invalid['image_content']['passed']=False;invalid['passed']=False
    result['qualification_with_image_content_failure']=qualification(invalid,1)
    invalid=copy.deepcopy(raw);invalid['origin_qualification']['passed']=False;invalid['passed']=False
    result['qualification_with_origin_qualification_failure']=qualification(invalid,1)
    result['mounted_evidence_dependencies']={p:(PREP/p).exists() for p in ['observer','contact-observer','allocator','allocator-models/manifest.json','turn-slip','turn-models/pure_turn.sdf','slip-models/manifest.json','test-startup/image_dds_guard.py']}
    paths=[PREP/'check_r5_v2_offset_round.py',PREP/'run_diagnosis_campaign.py',SCRIPTS/'check_r5_turn_slip_round.py',SCRIPTS/'check_r5_origin_round.py',SCRIPTS/'check_r5_runtime.py']
    result['executed_source_sha256']={str(p.relative_to(WORKSPACE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    (ROOT/'review-results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':main()
