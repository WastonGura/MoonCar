"""Workbench scene selection against the shipped Terrain V2 collision assets.

These are setup checks, not sensor or localization acceptance measurements.
"""
import http.client
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
from urllib.parse import quote
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
from simulation_experiment import SimulationExperiment, spawn_obstacles, survey_world, validate_task
from experiment_scenes import scene_metadata
from map_workbench import create_server
from autonomous_survey import AutonomousSurvey

PACKAGE = ROOT / 'ros2_ws/src/mooncar_gazebo'
V2_WORLD = PACKAGE / 'worlds/mooncar_survey_v2.sdf'
V2_ASSETS = PACKAGE / 'models/terrain_v2/survey'


def task(**changes):
    selected = dict(version=1, region=[-15, -15, 15, 15], start=[0, 0, 0],
                    scene='survey_v2', gui=False)
    selected.update(changes)
    return selected


def mesh_triangles(path):
    """Read the actual SDF collision fixture; do not regenerate terrain."""
    vertices, faces = [], []
    for line in path.read_text(encoding='ascii').splitlines():
        fields = line.split()
        if fields and fields[0] == 'v':
            vertices.append([float(v) for v in fields[1:4]])
        elif fields and fields[0] == 'f':
            faces.append([int(v.split('/')[0]) - 1 for v in fields[1:]])
    return np.asarray(vertices)[np.asarray(faces)]


class ExperimentSceneTests(unittest.TestCase):
    def test_native_v2_is_the_shipped_world_without_added_fixtures(self):
        selected = survey_world('survey_v2_native')
        self.assertEqual(ET.tostring(selected.getroot()), ET.tostring(ET.parse(V2_WORLD).getroot()))
        self.assertFalse(any(model.get('name').startswith('wb_localization_marker_')
                             for model in selected.findall('./world/model')))
        # The marker at (3, 7) exists only in the compatibility world. Native
        # spawn validation must not reject its empty, supported ground.
        selected_task = task(scene='survey_v2_native', start=[3, 7, 0])
        self.assertEqual(validate_task(selected_task), selected_task)
        with self.assertRaises(ValueError):
            validate_task(dict(selected_task, scene='survey_v2'))
        native = spawn_obstacles('survey_v2_native')
        compatibility = spawn_obstacles('survey_v2')
        self.assertEqual(len(compatibility), len(native) + 4)
        for actual, expected in zip(native, compatibility):
            np.testing.assert_array_equal(actual, expected)

    def test_v2_selects_shipped_world_without_replacing_its_physics_or_geometry(self):
        expected = ET.parse(V2_WORLD).find('world')
        selected = survey_world('survey_v2')
        world = selected.find('world')
        self.assertEqual(world.get('name'), 'mooncar_survey_v2')
        # Workbench localization markers are additions to the real V2 terrain.
        # Every shipped world element, especially physics and collision meshes,
        # must remain present rather than recreating V1 under a new scene name.
        elements = [ET.tostring(element) for element in world]
        for element in expected:
            with self.subTest(tag=element.tag, name=element.get('name')):
                self.assertIn(ET.tostring(element), elements)
        original_names = {model.get('name') for model in expected.findall('model')}
        additions = [model for model in world.findall('model') if model.get('name') not in original_names]
        self.assertEqual(len(additions), 4)
        for marker in additions:
            self.assertEqual(marker.findtext('static'), 'true')
            collision, visual = marker.find('.//collision/geometry'), marker.find('.//visual/geometry')
            self.assertIsNotNone(collision)
            self.assertIsNotNone(visual)
            self.assertEqual(ET.tostring(collision), ET.tostring(visual))
        self.assertIsNotNone(selected.find("./world/model[@name='ce4_crater_patch_ground']"))
        self.assertIsNone(selected.find("./world/model[@name='lunar_regolith_plane']"))

    def test_v2_task_round_trips_and_uses_its_actual_thirty_metre_bounds(self):
        selected = task()
        for scene in ('survey_v2', 'survey_v2_native'):
            with self.subTest(scene=scene):
                native_or_compatibility = dict(selected, scene=scene)
                self.assertEqual(validate_task(json.loads(json.dumps(native_or_compatibility))),
                                 native_or_compatibility)
        with np.load(V2_ASSETS / 'height_grid.npz') as grid:
            self.assertEqual([float(grid['x'][0]), float(grid['y'][0]),
                              float(grid['x'][-1]), float(grid['y'][-1])], selected['region'])
        for scene in ('survey_v2', 'survey_v2_native'):
            for region in ([-25, -25, 25, 25], [-15.1, -15, 15, 15], [-15, -15, 15, 15.1]):
                with self.subTest(scene=scene, region=region), self.assertRaises(ValueError):
                    validate_task(dict(selected, scene=scene, region=region))
        # A legacy saved task keeps the old 50 m scene and does not acquire V2.
        legacy = dict(version=1, region=[-25, -25, 25, 25], start=[3.5, 3, 0])
        self.assertEqual(validate_task(legacy), legacy)

    def test_v2_rejects_each_real_boulder_spawn(self):
        manifest = json.loads((V2_ASSETS / 'manifest.json').read_text(encoding='utf-8'))
        for boulder in manifest['boulders']:
            with self.subTest(boulder=boulder['name']), self.assertRaises(ValueError):
                validate_task(dict(task(), start=boulder['position_m'][:2] + [0]))
        originals = {model.get('name') for model in ET.parse(V2_WORLD).findall('./world/model')}
        for marker in survey_world('survey_v2').findall('./world/model'):
            if marker.get('name') in originals:
                continue
            x, y = [float(v) for v in marker.findtext('pose').split()[:2]]
            with self.subTest(marker=marker.get('name')), self.assertRaises(ValueError):
                validate_task(task(start=[x, y, 0]))

    def test_v2_requires_support_for_the_whole_vehicle_at_world_edges(self):
        for start in ([14.9, 0, 0], [-14.9, 0, 0], [0, 14.9, math.pi/2], [0, -14.9, math.pi/2]):
            with self.subTest(start=start), self.assertRaises(ValueError):
                validate_task(dict(task(), start=start))

    def test_v2_rejects_a_steep_triangle_from_the_measured_patch(self):
        triangles = mesh_triangles(V2_ASSETS / 'ce4_crater_patch.obj')
        normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        slope = np.degrees(np.arctan2(np.linalg.norm(normal[:, :2], axis=1), np.abs(normal[:, 2])))
        steepest = int(np.argmax(slope))
        self.assertGreater(slope[steepest], 20, 'The fixture must contain an actual steep collision face')
        x, y, _ = triangles[steepest].mean(axis=0)
        with self.assertRaises(ValueError):
            validate_task(dict(task(), start=[float(x), float(y), 0]))

    def test_scene_survives_task_download_and_same_origin_apply(self):
        selected = task(scene='survey_v2_native')
        with tempfile.TemporaryDirectory() as directory:
            simulation = Mock()
            simulation.apply.return_value = dict(accepted=True)
            server = create_server(directory, port=0, simulation=simulation)
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            client = http.client.HTTPConnection('127.0.0.1', server.server_port)
            try:
                client.request('GET', '/api/task?task=' + quote(json.dumps(selected)))
                response = client.getresponse()
                self.assertEqual(response.status, 200)
                downloaded = json.loads(response.read())
                self.assertEqual(downloaded, selected)
                client.request('POST', '/api/simulation/apply', json.dumps(downloaded),
                               {'Content-Type': 'application/json',
                                'Origin': f'http://127.0.0.1:{server.server_port}'})
                response = client.getresponse()
                self.assertEqual(response.status, 200)
                response.read()
                simulation.apply.assert_called_once_with(selected)
            finally:
                client.close()
                server.shutdown()
                server.server_close()
                worker.join(timeout=3)

    def test_invalid_v2_spawn_does_not_stop_the_current_experiment(self):
        simulation = SimulationExperiment.__new__(SimulationExperiment)
        simulation._stop = Mock()
        with self.assertRaises(ValueError):
            simulation.apply(dict(task(), start=[14.9, 0, 0]))
        simulation._stop.assert_not_called()

    def test_autonomy_uses_scene_surface_bounds_and_keeps_saved_task_unchanged(self):
        with np.load(V2_ASSETS / 'height_grid.npz') as grid:
            v2_bounds = [float(grid['x'][0]), float(grid['y'][0]),
                         float(grid['x'][-1]), float(grid['y'][-1])]
        for scene, bounds in [('survey_v2', v2_bounds), ('survey_v2_native', v2_bounds),
                              ('survey', [-25., -25., 25., 25.])]:
            with self.subTest(scene=scene), tempfile.TemporaryDirectory() as directory:
                motion = Mock()
                motion.lock = threading.RLock()
                motion.get_clock.return_value.now.return_value.nanoseconds = 0
                motion.quality = {}
                motion.quality_received = __import__('time').monotonic()
                motion.state.return_value = dict(active=False, ready=True, pose=dict(x=0., y=0., yaw=0.))
                motion.spin_client.server_is_ready.return_value = True
                survey = AutonomousSurvey(motion)
                saved_task = dict(id='test', start=[0, 0, 0], scene=scene, region=[-4, -4, 7, 4])
                (Path(directory) / 'test').mkdir()
                with patch.object(survey, 'run_recorded'):
                    self.assertEqual(survey.start(dict(experiment='test', budget_seconds=15, radius=6),
                                                  saved_task, directory), dict(accepted=True))
                    survey.worker.join(timeout=3)
                self.assertEqual(survey.task['region'], bounds)
                self.assertEqual(survey.task['scene'], scene)
                self.assertEqual(saved_task['region'], [-4, -4, 7, 4])
                motion.start.assert_not_called()
                motion.turn.assert_not_called()

    def test_source_metadata_describes_the_actual_localization_backend(self):
        terrain = scene_metadata('survey_v2_native', 'terrain')
        self.assertIn('3D 深度与 IMU', terrain['source_description'])
        self.assertIn('无追加定位标柱', terrain['source_description'])
        self.assertEqual(terrain['localization_fixture_profile'], 'none')
        self.assertEqual(terrain['localization_mode'], 'terrain')
        for mode in ('direct', 'ekf', None):
            with self.subTest(mode=mode):
                source = scene_metadata('survey_v2_native', mode)
                self.assertIn('IMU', source['source_description'])
                self.assertNotIn('3D 深度与 IMU 地形定位', source['source_description'])
        with self.assertRaises(ValueError):
            scene_metadata('survey_v2_native', 'unknown')

    def test_apply_native_v2_launches_terrain_and_saves_truthful_scene_metadata(self):
        selected = task(scene='survey_v2_native', start=[3, 7, 0])
        with tempfile.TemporaryDirectory() as directory:
            simulation = SimulationExperiment.__new__(SimulationExperiment)
            simulation.lock = threading.Lock()
            simulation.worker = None
            simulation.directory = Path(directory)
            simulation._stop = Mock()
            simulation.ros_domain = '132'
            simulation.laser_odometry = True
            simulation.laser_odometry_mode = 'terrain'
            simulation.log = None
            with patch('simulation_experiment.subprocess.Popen') as launch:
                try:
                    self.assertEqual(simulation.apply(selected), dict(accepted=True))
                    simulation.worker.join(timeout=15)
                    self.assertFalse(simulation.worker.is_alive())
                    self.assertEqual(simulation.phase, 'starting', getattr(simulation, 'error', ''))
                    launch.assert_called_once()
                    arguments = dict(value.split(':=', 1) for value in launch.call_args.args[0] if ':=' in value)
                    self.assertEqual(arguments['world_name'], 'mooncar_survey_v2')
                    self.assertEqual(arguments['laser_odometry_mode'], 'terrain')
                    self.assertEqual(arguments['render_engine'], 'ogre2')
                    self.assertIn('3D 深度与 IMU', arguments['source_description'])
                    folder = Path(arguments['output_directory'])
                    self.assertEqual(ET.tostring(ET.parse(folder / 'survey_yard.sdf').getroot()),
                                     ET.tostring(ET.parse(V2_WORLD).getroot()))
                    setup = json.loads((folder / 'scene-setup.json').read_text(encoding='utf-8'))
                    self.assertEqual(setup['localization_markers'], [])
                    self.assertEqual(setup['localization_fixture_profile'], 'none')
                    self.assertEqual(setup['localization_mode'], 'terrain')
                    self.assertFalse(setup['truth_inputs_to_navigation'])
                    self.assertEqual(json.loads((folder / 'task.json').read_text()), selected)
                    gui = ET.fromstring('<gui>' + (folder / 'gui.config').read_text() + '</gui>')
                    self.assertEqual(gui.findtext("plugin[@filename='MinimalScene']/engine"), 'ogre2')
                finally:
                    if simulation.log is not None:
                        simulation.log.close()

    def test_apply_saves_real_v2_and_passes_world_height_and_direct_selection(self):
        selected = task(start=[-10, -10, 0])
        # This real, gently sloped area lies below zero. A hard-coded flat-world
        # Z=0 (or old world_name=lunar_yard) would be the wrong launch selection.
        triangles = mesh_triangles(V2_ASSETS / 'regolith_passable.obj')
        nearby = triangles[(triangles[:, :, 0].max(axis=1) >= -10.53)
                           & (triangles[:, :, 0].min(axis=1) <= -9.47)
                           & (triangles[:, :, 1].max(axis=1) >= -10.46)
                           & (triangles[:, :, 1].min(axis=1) <= -9.54)]
        self.assertGreater(len(nearby), 0)
        with tempfile.TemporaryDirectory() as directory:
            simulation = SimulationExperiment.__new__(SimulationExperiment)
            simulation.lock = threading.Lock()
            simulation.worker = None
            simulation.directory = Path(directory)
            simulation._stop = Mock()
            simulation.ros_domain = '132'
            simulation.laser_odometry = True
            simulation.laser_odometry_mode = 'direct'
            simulation.log = None
            with patch('simulation_experiment.subprocess.Popen') as launch:
                try:
                    self.assertEqual(simulation.apply(selected), dict(accepted=True))
                    simulation.worker.join(timeout=15)
                    self.assertFalse(simulation.worker.is_alive())
                    self.assertEqual(simulation.phase, 'starting', getattr(simulation, 'error', ''))
                    launch.assert_called_once()
                    command = launch.call_args.args[0]
                    arguments = dict(argument.split(':=', 1) for argument in command if ':=' in argument)
                    self.assertEqual(arguments['world_name'], 'mooncar_survey_v2')
                    self.assertEqual(arguments['laser_odometry'], 'true')
                    self.assertEqual(arguments['laser_odometry_mode'], 'direct')
                    self.assertIn('Terrain V2', arguments['source_description'])
                    self.assertIn('定位标柱', arguments['source_description'])
                    initial_z = float(arguments['initial_z'])
                    self.assertLess(initial_z, 0)
                    self.assertGreaterEqual(initial_z, float(nearby[:, :, 2].min()))
                    self.assertLessEqual(initial_z, float(nearby[:, :, 2].max()))
                    folder = Path(arguments['output_directory'])
                    self.assertEqual(json.loads((folder / 'task.json').read_text(encoding='utf-8')), selected)
                    saved = ET.parse(folder / 'survey_yard.sdf')
                    self.assertEqual(ET.tostring(saved.getroot()),
                                     ET.tostring(survey_world('survey_v2').getroot()))
                    self.assertEqual(simulation.current['scene'], 'survey_v2')
                    setup = json.loads((folder / 'scene-setup.json').read_text(encoding='utf-8'))
                    self.assertTrue(setup['setup_only'])
                    self.assertFalse(setup['truth_inputs_to_navigation'])
                    self.assertEqual(len(setup['localization_markers']), 4)
                    gui = ET.fromstring('<gui>' + (folder / 'gui.config').read_text(encoding='utf-8') + '</gui>')
                    self.assertEqual(gui.findtext("plugin[@filename='MinimalScene']/engine"), 'ogre2')
                finally:
                    if simulation.log is not None:
                        simulation.log.close()


if __name__ == '__main__':
    unittest.main()
