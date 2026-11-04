"""Goal-local precision resources and parent arrival transaction contracts.

Actual Motion.start/helper AST executes with action/message doubles; no ROS
process runs. Parent proof is mocked only to test its required phase ordering.
"""
import ast
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import sys
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
ROOT=next(p for p in HERE.parents if (p/'scripts/autonomous_survey.py').is_file())
NAVIGATION=ROOT/'ros2_ws/src/mooncar_navigation'
MOTION=HERE/'experiment_motion.py'if(HERE/'experiment_motion.py').is_file()else ROOT/'scripts/experiment_motion.py'
LAUNCH=HERE/'terrain_navigation.launch.py'if(HERE/'terrain_navigation.launch.py').is_file()else NAVIGATION/'launch/terrain_navigation.launch.py'
ASSETS=HERE/'config/behavior_trees'if(HERE/'config/behavior_trees').is_dir()else NAVIGATION/'config/behavior_trees'
CMAKE=HERE/'CMakeLists.txt'if(HERE/'CMakeLists.txt').is_file()else NAVIGATION/'CMakeLists.txt'
sys.path[:0]=[str(HERE),str(ROOT/'scripts'),str(ROOT/'ros2_ws/src/mooncar_terrain/tests')]
import autonomous_survey
import test_localization_revisit_displacement as displacement


def actual_function(path,name,class_name=None):
    tree=ast.parse(path.read_text(encoding='utf-8-sig'))
    scope=tree.body if class_name is None else next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name==class_name).body
    node=next(n for n in scope if isinstance(n,ast.FunctionDef)and n.name==name)
    return node


def compile_function(node,namespace):
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),'actual_precision_contract','exec'),namespace)
    return namespace[node.name]


class Goal:
    def __init__(self):
        self.behavior_tree=''
        self.pose=NS(header=NS(frame_id=''),pose=NS(position=NS(x=0.,y=0.),orientation=NS(z=0.,w=1.)))


class PrecisionNavigationResourceTests(unittest.TestCase):
    def fixture(self):
        namespace=dict(math=math,secrets=secrets,time=time,NavigateToPose=NS(Goal=Goal))
        namespace['within_region']=compile_function(actual_function(MOTION,'within_region'),dict(math=math))
        start=compile_function(actual_function(MOTION,'start','ExperimentMotion'),namespace)
        motion=NS(lock=threading.RLock(),active=False,phase='idle',state=Mock(return_value=dict(ready=True)),
            check_goal=Mock(),pose=Mock(return_value=dict(x=0.,y=0.,z=0.,yaw=0.)),check_start=Mock(),
            activate_localization_action=Mock(),get_clock=Mock(),client=object(),on_feedback=Mock())
        motion.get_clock.return_value.now.return_value.nanoseconds=10_000_000_000
        def send(client,request,token,kind,callback):
            motion.goal_dispatch=dict(request_id='actual-fake-action',kind=kind)
        motion.send_goal=Mock(side_effect=send)
        return start,motion,dict(experiment='test',goal=[1.,0.]),dict(id='test',region=[-15.,-15.,15.,15.])

    def packages(self,share):
        packages=ModuleType('ament_index_python.packages')
        packages.get_package_share_directory=Mock(return_value=str(share))
        parent=ModuleType('ament_index_python');parent.packages=packages
        return packages,patch.dict(sys.modules,{'ament_index_python':parent,'ament_index_python.packages':packages})

    def test_real_start_keeps_ordinary_default_and_precise_uses_installed_ament_tree(self):
        start,motion,body,task=self.fixture()
        with patch.dict(sys.modules,{'ament_index_python.packages':None}):start(motion,body,task)
        ordinary=motion.send_goal.call_args.args[1]
        self.assertEqual(ordinary.behavior_tree,'')
        with tempfile.TemporaryDirectory()as folder:
            share=Path(folder);installed=share/'config/behavior_trees/navigate_to_pose_revisit.xml'
            installed.parent.mkdir(parents=True);installed.write_bytes((ASSETS/installed.name).read_bytes())
            start,motion,body,task=self.fixture();packages,context=self.packages(share)
            with context:start(motion,body,task,precise_revisit=True)
            packages.get_package_share_directory.assert_called_once_with('mooncar_navigation')
            request=motion.send_goal.call_args.args[1]
            self.assertEqual(request.behavior_tree,str(installed))
            self.assertEqual((request.pose.header.frame_id,request.pose.pose.position.x),('map',1.))
            self.assertEqual(motion.goal_dispatch['goal_checker_id'],'revisit_goal_checker')
            self.assertEqual(motion.goal_dispatch['position_tolerance_m'],.05)

    def test_precision_is_an_internal_strict_bool_keyword_not_a_body_field(self):
        start,motion,body,task=self.fixture()
        with self.assertRaises(ValueError):start(motion,dict(body,precise_revisit=True),task)
        for bad in (1,'true',None):
            with self.subTest(bad=bad),self.assertRaises(ValueError):start(motion,body,task,precise_revisit=bad)
        motion.activate_localization_action.assert_not_called();motion.send_goal.assert_not_called()

    def test_missing_malformed_or_wrong_checker_tree_cannot_activate_a_lease(self):
        for invalid in (None,'<root>', '<root><FollowPath goal_checker_id="goal_checker"/></root>',
                        '<root><FollowPath goal_checker_id="revisit_goal_checker"/><FollowPath goal_checker_id="revisit_goal_checker"/></root>'):
            with self.subTest(invalid=invalid),tempfile.TemporaryDirectory()as folder:
                share=Path(folder);installed=share/'config/behavior_trees/navigate_to_pose_revisit.xml'
                installed.parent.mkdir(parents=True)
                if invalid is not None:installed.write_text(invalid)
                start,motion,body,task=self.fixture();_,context=self.packages(share)
                with context,self.assertRaises(ValueError):start(motion,body,task,precise_revisit=True)
                motion.activate_localization_action.assert_not_called();motion.send_goal.assert_not_called()

    def test_two_complete_trees_only_change_explicit_goal_and_controller_selectors(self):
        def shape(node):return dict(tag=node.tag,attributes=dict(node.attrib),text=(node.text or '').strip(),children=[shape(child)for child in node])
        normalized=[]
        for name,checker in (('navigate_to_pose_workbench.xml','goal_checker'),('navigate_to_pose_revisit.xml','revisit_goal_checker')):
            tree=ET.fromstring((ASSETS/name).read_bytes());followers=tree.findall('.//FollowPath')
            self.assertEqual(len(followers),1);self.assertEqual(followers[0].attrib.pop('goal_checker_id'),checker)
            expected='RevisitFollowPath'if checker=='revisit_goal_checker'else'{selected_controller}'
            self.assertEqual(followers[0].get('controller_id'),expected)
            # Normalize only after proving the dedicated literal selection.
            followers[0].set('controller_id','{selected_controller}')
            self.assertEqual(sum(1 for _ in tree.iter()),30)
            normalized.append(json.dumps(shape(tree),sort_keys=True,separators=(',',':')).encode())
        self.assertEqual(normalized[0],normalized[1])
        # Full installed Jazzy source structure, not merely an empty FollowPath.
        self.assertEqual(hashlib.sha256(normalized[0]).hexdigest(),'95da57c55a46da5a5e342b74bab861fbdc59e62be5c871bf1e87bf1ad4b59943')
        text=CMAKE.read_text();directory=text.split('DIRECTORY',1)[1].split('DESTINATION',1)[0].split()
        self.assertIn('config',directory);self.assertIn('share/${PROJECT_NAME}',text)

    def test_actual_launch_helper_keeps_ordinary_point25_and_separate_point05(self):
        helper=compile_function(actual_function(LAUNCH,'configure_workbench_goal_checkers'),dict(os=os))
        configure_controller=compile_function(actual_function(LAUNCH,'configure_revisit_controller'),dict(copy=copy))
        ordinary=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            desired_linear_vel=.16,lookahead_dist=.55,min_lookahead_dist=.30,max_lookahead_dist=.80,
            min_approach_linear_velocity=.05,regulated_linear_scaling_min_speed=.05,
            rotate_to_heading_angular_vel=.8,rotate_to_heading_min_angle=.785,
            use_collision_detection=True,max_allowed_time_to_collision_up_to_carrot=1.5,
            max_angular_accel=8.,use_velocity_scaled_lookahead_dist=True,
            use_regulated_linear_velocity_scaling=True,use_cost_regulated_linear_velocity_scaling=True,
            inherited_nested_guard=dict(values=[1,2,3]))
        ordinary_before=copy.deepcopy(ordinary)
        params=dict(controller_server=dict(ros__parameters=dict(goal_checker=dict(plugin='old',stateful=True,xy_goal_tolerance=.08),
            progress_checker=dict(required_movement_radius=.1),FollowPath=ordinary,
            controller_plugins=['FollowPath','ExtraController'],ExtraController=dict(plugin='unchanged'))),
            bt_navigator=dict(ros__parameters={}),unrelated=dict(value=17))
        before=copy.deepcopy(params['controller_server']['ros__parameters']['progress_checker'])
        helper(params,'installed/share/mooncar_navigation')
        configure_controller(params)
        controller=params['controller_server']['ros__parameters']
        self.assertEqual(controller['goal_checker_plugins'],['goal_checker','revisit_goal_checker'])
        for name,tolerance in (('goal_checker',.25),('revisit_goal_checker',.05)):
            self.assertEqual(controller[name]['plugin'],'nav2_controller::PositionGoalChecker')
            self.assertEqual(controller[name]['xy_goal_tolerance'],tolerance)
        self.assertEqual(controller['progress_checker'],before);self.assertEqual(params['unrelated'],dict(value=17))
        self.assertEqual(params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'],
                         os.path.join('installed/share/mooncar_navigation','config','behavior_trees','navigate_to_pose_workbench.xml'))
        overrides=dict(desired_linear_vel=.06,lookahead_dist=.15,min_lookahead_dist=.10,max_lookahead_dist=.20,
            min_approach_linear_velocity=.02,regulated_linear_scaling_min_speed=.02,
            rotate_to_heading_angular_vel=.35,rotate_to_heading_min_angle=.35)
        self.assertEqual(controller['FollowPath'],ordinary_before)
        self.assertIsNot(controller['RevisitFollowPath'],controller['FollowPath'])
        self.assertEqual(controller['RevisitFollowPath'],dict(ordinary_before,**overrides))
        self.assertEqual(controller['controller_plugins'],['FollowPath','ExtraController','RevisitFollowPath'])
        self.assertEqual(controller['ExtraController'],dict(plugin='unchanged'))
        for key,value in ordinary_before.items():
            if key not in overrides:self.assertEqual(controller['RevisitFollowPath'][key],value)
        controller['RevisitFollowPath']['inherited_nested_guard']['values'].append(4)
        self.assertEqual(controller['FollowPath'],ordinary_before)
        configure_controller(params)
        self.assertEqual(controller['controller_plugins'].count('RevisitFollowPath'),1)
        self.assertEqual(controller['FollowPath'],ordinary_before)


class PrecisionParentTransactionTests(unittest.TestCase):
    def setUp(self):
        self.original=displacement.RevisitDisplacementTests()
        self.original.setUp();self.addCleanup(self.original.doCleanups)

    def test_execute_passes_keyword_only_for_explicit_precise_internal_decision(self):
        for precise in (False,True):
            with self.subTest(precise=precise):
                survey,motion,_=self.original.original.fixture()
                motion.start.return_value=dict(token='new-token');motion.token='new-token'
                motion.state.return_value=dict(active=False,phase='succeeded',diagnostics=dict(goal_request_id='new',goal_request_kind='navigate_to_pose'))
                decision=dict(x=.1,y=0.,kind='advance')
                if precise:decision.update(localization_revisit=True,precise_revisit=True)
                with patch.object(survey,'healthy',return_value=True):self.assertTrue(survey.execute(decision))
                self.assertEqual(motion.start.call_args.kwargs,dict(precise_revisit=True)if precise else{})
                self.assertEqual(motion.start.call_args.args[0],dict(experiment='test',goal=[.1,0.]))

    def test_point10_or_point216_initial_distance_dispatches_precise_goal_before_proof(self):
        for distance in (.10,.216):
            with self.subTest(distance=distance):
                survey,motion,checkpoint,calls=self.original.setup_actor(first_drift=0.)
                survey.revisit_policy.plan.return_value['goals']=[checkpoint]
                motion.pose.return_value=dict(x=distance,y=0.,yaw=0.)
                with patch.object(survey,'healthy',return_value=True):self.assertTrue(survey.perform_localization_revisit())
                self.assertEqual(calls[0]['kind'],'navigate_to_pose')
                self.assertIs(calls[0]['decision']['precise_revisit'],True)
                self.assertEqual(sum(r['kind']=='navigate_to_pose'for r in calls),1)

    def test_point10_or_point216_postview_displacement_needs_precise_correction(self):
        for distance in (.10,.216):
            with self.subTest(distance=distance):
                survey,motion,checkpoint,calls=self.original.setup_actor(first_drift=distance)
                def prove(*args):
                    self.assertEqual([r['kind']for r in calls],['spin','gimbal','navigate_to_pose','gimbal'])
                    self.assertLessEqual(math.hypot(motion.pose()['x']-checkpoint['x'],motion.pose()['y']-checkpoint['y']),.05)
                    return True
                survey.revisit_policy.prove.side_effect=prove
                with patch.object(survey,'healthy',return_value=True):self.assertTrue(survey.perform_localization_revisit())
                correction=next(r for r in calls if r['kind']=='navigate_to_pose')
                self.assertIs(correction['decision']['precise_revisit'],True)
                self.assertEqual(survey.revisit_policy.attempts,2)

    def test_residual_point10_after_correction_or_second_view_stops_without_proof(self):
        for kind in ('arrival','second_view'):
            with self.subTest(kind=kind):
                kwargs=dict(arrival=.10)if kind=='arrival'else dict(second_drift=.10)
                survey,motion,checkpoint,calls=self.original.setup_actor(**kwargs)
                with patch.object(survey,'healthy',return_value=True),patch.object(survey,'stop'):
                    self.assertFalse(survey.perform_localization_revisit())
                self.assertEqual(sum(r['kind']=='navigate_to_pose'for r in calls),1)
                survey.revisit_policy.prove.assert_not_called()

    def test_original_four_attempt_and_quality_fences_precede_precise_correction(self):
        for kind in ('attempt_limit','quality_fault'):
            with self.subTest(kind=kind):
                survey,motion,checkpoint,record,calls=self.original.direct_actor()
                if kind=='attempt_limit':survey.revisit_policy.attempts=4
                with patch.object(survey,'healthy',return_value=kind!='quality_fault'),patch.object(survey,'stop'):
                    self.assertFalse(survey.correct_revisit_view_displacement(checkpoint,record))
                survey.execute.assert_not_called();survey.revisit_policy.prove.assert_not_called()


if __name__=='__main__':unittest.main()
