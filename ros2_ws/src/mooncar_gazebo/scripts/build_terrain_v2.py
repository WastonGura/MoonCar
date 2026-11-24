#!/usr/bin/env python3
"""Build three versioned lunar terrain worlds from one continuous surface per scene.

Visual and collision meshes use the same vertices. Material classes split the
surface into separate meshes so each class can also have a contact coefficient.
The survey world includes one measured DTM patch. Synthetic slopes and crater
scales use cited observations; profiles, placement and contact remain hypotheses.
"""

from __future__ import annotations

import hashlib
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from generate_terrain_materials import bake_scene
from terrain_scene_detail import SEED, rock_variants, write_rocks, pebbles, sample_ground


SOURCE_PACKAGE = Path(__file__).resolve().parents[1]
PACKAGE = SOURCE_PACKAGE
SCENES = {"ramp": (16.0, 0.25), "curve": (16.0, 0.25), "survey": (30.0, 0.30)}
CURVE_CRATERS = ((1.5, 4.0, 3.05), (5.5, -5.5, 5.6))
# Ding et al. 2024, p.3: maximum slope on one Yutu-2 route, not a global lunar limit.
RAMP_SLOPE_DEG = 8.86
RAMP_LENGTH = 6.0  # Engineering layout, retained.
# Guo et al. 2021, pp.3-4, region R3: D ~4.5 m, depth/diameter ~0.07.
SURVEY_CRATER_RADIUS = 4.5 / 2
SURVEY_CRATER_DEPTH_RATIO = 0.07
# Ding et al. 2024, PDF p.7, target boulder at Longji: height approximately 0.41 m.
# One reference obstacle, not a mean/maximum for every rock or a shape reconstruction.
CURVE_REFERENCE_ROCK_HEIGHT = 0.41
PATCH_CENTER = (4.8, 3.6)
PATCH_SIZE = 4.8
PATCH_STEP = 0.05
PATCH_SOURCE = PACKAGE / "data/terrain_sources/ce4_fresh_crater_5cm.npz"
# Guo 2021 / Zenodo 4971121, valid 6 m crop at original scale.
BACKGROUND_CENTER = (6.0, -5.1)  # Coarse-cell alignment; engineered placement.
BACKGROUND_SIZE = 6.0
BACKGROUND_SOURCE = PACKAGE / "data/terrain_sources/ce4_background_5cm.npz"
# TODO(terrain:ROCK-LAYOUT): XY sizes, shape, burial and positions are engineered.
# Curve boulder_01's rz is derived from its cited exposed height when building.
BOULDERS = {
    "ramp": [(-2.8, 3.8, 0.48, 0.40, 0.30), (4.3, -4.0, 0.62, 0.49, 0.38)],
    "curve": [(-0.7, 6.0, 0.55, 0.42, None), (7.0, -0.1, 0.60, 0.50, 0.36)],
    "survey": [(-3.2, 4.5, 0.215, 0.215, 0.150), (1.7, 7.0, 0.170, 0.170, 0.120),
               (7.2, 3.0, 0.135, 0.135, 0.100), (-7.4, -1.5, 0.130, 0.130, 0.100),
               (4.0, -7.4, 0.120, 0.120, 0.090)],
}
# TODO(terrain:CONTACT): these rigid-contact coefficients are not lunar-soil
# internal friction angles or DEM particle coefficients; direct matching is pending.
CLASSES = {
    "regolith_passable": {"id": 0, "mu": 0.80, "mu2": 0.72},
    "sand_soft": {"id": 1, "mu": 0.52, "mu2": 0.48},
    "rubble_gravel": {"id": 2, "mu": 0.90, "mu2": 0.82},
    "rock_boulder": {"id": 3, "mu": 1.00, "mu2": 0.92},
    "crater_floor": {"id": 4, "mu": 0.64, "mu2": 0.58},
}


def smoothstep(value):
    value = np.clip(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def base_relief(x, y):
    # TODO(terrain:BACKGROUND): amplitudes/wavelengths are engineering values.
    # Guo's ~4 mm RMS uses 125 mm windows, unresolved on these 250/300 mm meshes.
    distance = np.hypot(x, y)
    fade = smoothstep((distance - 0.8) / 1.2)
    return fade * (0.013 * np.sin(0.8 * x + 0.3 * y) * np.sin(0.7 * y)
                   + 0.006 * np.sin(1.9 * x - 1.3 * y))


def ramp_height():
    """Convert the cited grade to height using the actual mesh's center route."""
    size, step = SCENES["ramp"]
    xs = np.linspace(-size / 2, size / 2, round(size / step) + 1)
    ground_grade = np.diff(base_relief(xs, 0.0)) / np.diff(xs)
    unit_grade = np.diff(smoothstep((xs - 1.0) / RAMP_LENGTH)) / np.diff(xs)
    rising = unit_grade > 0
    target = math.tan(math.radians(RAMP_SLOPE_DEG))
    assert np.max(np.abs(ground_grade)) < target
    return float(np.min((target - ground_grade[rising]) / unit_grade[rising]))


RAMP_HEIGHT = ramp_height()


def crater(x, y, cx, cy, radius, depth):
    # TODO(terrain:CRATER-PROFILE): bowl/rim functions are synthetic; cited
    # diameter/depth ratios do not establish these exact shape coefficients.
    r = np.hypot(x - cx, y - cy)
    bowl = -depth * np.exp(-((r / (0.62 * radius)) ** 4))
    rim = 0.45 * depth * np.exp(-(((r - radius) / (0.22 * radius)) ** 2))
    return bowl + rim


def terrain(scene, x, y):
    z = base_relief(x, y)
    if scene == "ramp":
        ramp = smoothstep((x - 1.0) / RAMP_LENGTH)
        side = smoothstep((7.5 - np.abs(y)) / 6.0)
        return z + RAMP_HEIGHT * ramp * side
    if scene == "curve":
        for cx, cy, radius in CURVE_CRATERS:
            # The 6.1 m and 11.2 m diameters are observed near Yutu-2's route;
            # their local depth here follows the separate, typical d/D=0.061 study.
            z += crater(x, y, cx, cy, radius, 0.061 * (2 * radius) / 1.45)
        return z
    if scene == "survey":
        z += crater(x, y, -6.0, 7.0, SURVEY_CRATER_RADIUS,
                    SURVEY_CRATER_DEPTH_RATIO * (2 * SURVEY_CRATER_RADIUS) / 1.45)
        # TODO(terrain:MOUNDS): amplitudes, widths and positions are engineered.
        z += 0.28 * np.exp(-(((x - 6.0) / 1.3) ** 2 + ((y + 5.0) / 0.9) ** 2))
        z += 0.20 * np.exp(-(((x + 5.0) / 1.5) ** 2 + ((y + 6.0) / 1.0) ** 2))
        return z
    raise ValueError(scene)


def material_ids(scene, x, y, z):
    result = np.zeros(x.shape, dtype=np.uint8)
    if scene == "ramp":
        result[(x > 1.6) & (x < 6.0) & (np.abs(y) < 1.25)] = 1
        result[(x > 1.4) & (x < 6.3) & (np.abs(y) >= 1.25) & (np.abs(y) < 3.1)] = 2
        result[(x > 4.5) & (y > 4.0)] = 3
    elif scene == "curve":
        for bx, by, radius in CURVE_CRATERS:
            distance = np.hypot(x - bx, y - by)
            result[distance < 1.22 * radius] = 2
            result[distance < 0.72 * radius] = 4
    elif scene == "survey":
        result[(((x + 5.0) / 2.6) ** 2 + ((y + 3.0) / 2.0) ** 2) < 1.0] = 1
        result[(((x - 3.0) / 2.4) ** 2 + ((y + 5.0) / 2.1) ** 2) < 1.0] = 2
        for cx, cy, radius in ((-6.0, 7.0, SURVEY_CRATER_RADIUS),):
            distance = np.hypot(x - cx, y - cy)
            result[(distance > 0.72 * radius) & (distance < 1.20 * radius)] = 2
            result[distance < 0.72 * radius] = 4
        result[z > 0.19] = 3
    else:
        raise ValueError(scene)
    result[np.hypot(x, y) < 0.9] = 0
    return result


def write_obj(path, xs, ys, z, normals, labels, class_id):
    cells = np.argwhere(labels == class_id)
    if len(cells) == 0:
        return 0
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# MoonCar terrain v2: common visual and collision surface\n")
        for cell_index, (row, col) in enumerate(cells):
            corners = ((row, col), (row, col + 1), (row + 1, col + 1),
                       (row + 1, col))
            for j, i in corners:
                stream.write(f"v {xs[i]:.6f} {ys[j]:.6f} {z[j, i]:.6f}\n")
            for j, i in corners:
                size = SCENES[path.parent.name][0]
                stream.write(f"vt {(xs[i] + size/2) / size:.8f} {(ys[j] + size/2) / size:.8f}\n")
            for j, i in corners:
                nx, ny, nz = normals[j, i]
                stream.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")
            base = 4 * cell_index + 1
            stream.write(f"f {base}/{base}/{base} {base+1}/{base+1}/{base+1} "
                         f"{base+2}/{base+2}/{base+2}\n")
            stream.write(f"f {base}/{base}/{base} {base+2}/{base+2}/{base+2} "
                         f"{base+3}/{base+3}/{base+3}\n")
    return len(cells)


def write_boulder_obj(path):
    phi = (1.0 + math.sqrt(5.0)) / 2.0
    vertices = np.array([(-1, phi, 0), (1, phi, 0), (-1, -phi, 0), (1, -phi, 0),
                         (0, -1, phi), (0, 1, phi), (0, -1, -phi), (0, 1, -phi),
                         (phi, 0, -1), (phi, 0, 1), (-phi, 0, -1), (-phi, 0, 1)],
                        dtype=float)
    vertices /= np.linalg.norm(vertices, axis=1, keepdims=True)
    vertices *= (1.0 + 0.11 * np.sin(np.arange(len(vertices)) * 13.7 + 0.5))[:, None]
    faces = ((0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
             (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
             (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1))
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# Shared faceted MoonCar boulder: visual and collision mesh\n")
        for face_index, face in enumerate(faces):
            points = vertices[list(face)]
            normal = np.cross(points[1] - points[0], points[2] - points[0])
            if np.dot(normal, points.mean(axis=0)) < 0:
                points = points[[0, 2, 1]]
                normal = -normal
            normal /= np.linalg.norm(normal)
            for vx, vy, vz in points:
                stream.write(f"v {vx:.6f} {vy:.6f} {vz:.6f}\n")
                stream.write(f"vt {0.5 + math.atan2(vy, vx) / (2 * math.pi):.6f} "
                             f"{math.acos(np.clip(vz, -1, 1)) / math.pi:.6f}\n")
                stream.write(f"vn {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}\n")
            base = 3 * face_index + 1
            stream.write(f"f {base}/{base}/{base} {base+1}/{base+1}/{base+1} "
                         f"{base+2}/{base+2}/{base+2}\n")


def model_xml(scene, name, config):
    uri = f"model://mooncar_gazebo/models/terrain_v2/{scene}/{name}.obj"
    tex = f"model://mooncar_gazebo/materials/textures/{config.get('appearance', scene+'_surface')}"
    return f"""    <model name="{name}_ground">
      <static>true</static>
      <link name="surface">
        <collision name="terrain_collision">
          <geometry><mesh><uri>{uri}</uri></mesh></geometry>
          <surface><friction>
            <ode><mu>{config['mu']}</mu><mu2>{config['mu2']}</mu2></ode>
            <bullet><friction>{config['mu']}</friction><friction2>{config['mu2']}</friction2></bullet>
          </friction></surface>
        </collision>
        <visual name="terrain_visual">
          <geometry><mesh><uri>{uri}</uri></mesh></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal>
              <albedo_map>{tex}_albedo.png</albedo_map>
              <normal_map>{tex}_normal.png</normal_map>
              <roughness_map>{tex}_roughness.png</roughness_map>
              <metalness>0.0</metalness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def boulder_xml(index, x, y, z, rx, ry, rz, variant):
    uri = f"model://mooncar_gazebo/models/terrain_v2/boulder_{variant:02d}.obj"
    tex = "model://mooncar_gazebo/materials/textures/rock_boulder"
    return f"""    <model name="boulder_{index:02d}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {0.47 * index:.3f}</pose>
      <link name="rock">
        <collision name="rock_collision">
          <geometry><mesh><uri>{uri}</uri><scale>{rx} {ry} {rz}</scale></mesh></geometry>
          <surface><friction><ode><mu>1.0</mu></ode><bullet><friction>1.0</friction></bullet></friction></surface>
        </collision>
        <visual name="rock_visual">
          <geometry><mesh><uri>{uri}</uri><scale>{rx} {ry} {rz}</scale></mesh></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>{tex}_albedo.png</albedo_map>
              <normal_map>{tex}_normal.png</normal_map>
              <roughness_map>{tex}_roughness.png</roughness_map>
              <metalness>0.0</metalness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


PHYSICS_BACKEND = 'dart'
PHYSICS_STEP = .001
PHYSICS_ENGINES = {
    'bullet': 'gz-physics-bullet-featherstone-plugin',
    'dart': 'libmooncar-physics7-dartsim.so',
}


def world_xml(scene, models):
    name = f"mooncar_{scene}_v2"
    return f"""<?xml version="1.0" encoding="utf-8"?>
<sdf version="1.10">
  <world name="{name}">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
      <engine><filename>{PHYSICS_ENGINES[PHYSICS_BACKEND]}</filename></engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <physics name="mooncar_v2_physics" type="ode">
      <max_step_size>{PHYSICS_STEP:g}</max_step_size><real_time_factor>1</real_time_factor>
    </physics>
    <gravity>0 0 -1.62</gravity>
    <scene><ambient>0.22 0.22 0.24</ambient><background>0.11 0.12 0.15</background><shadows>true</shadows></scene>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.92 0.86 1</diffuse>
      <specular>0.20 0.20 0.20 1</specular>
      <direction>-0.45 -0.30 -0.84</direction>
      <cast_shadows>true</cast_shadows>
    </light>
{chr(10).join(models)}
  </world>
</sdf>
"""


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def measured_patch(spawn_z):
    with np.load(PATCH_SOURCE) as source:
        local_x, local_y, residual = source["x"], source["y"], source["z"]
        assert source["source_md5"].item() == "39a21d550aa9376c2497b26324bb76b9"
    assert residual.shape == (97, 97) and abs(local_x[1] - local_x[0] - PATCH_STEP) < 1e-9
    px, py = np.meshgrid(PATCH_CENTER[0] + local_x, PATCH_CENTER[1] + local_y)
    lx, ly = np.meshgrid(local_x, local_y)
    edge_distance = np.minimum(PATCH_SIZE / 2 - np.abs(lx), PATCH_SIZE / 2 - np.abs(ly))
    feather = smoothstep(edge_distance / 0.35)
    pz = terrain("survey", px, py) - spawn_z + residual * feather
    border = np.zeros_like(pz, dtype=bool)
    border[[0, -1], :] = True
    border[:, [0, -1]] = True
    assert np.max(np.abs(pz[border] - (terrain("survey", px, py)[border] - spawn_z))) < 1e-9
    return PATCH_CENTER[0] + local_x, PATCH_CENTER[1] + local_y, pz, residual


def build(scene):
    size, step = SCENES[scene]
    count = round(size / step)
    xs = np.linspace(-size / 2, size / 2, count + 1)
    ys = np.linspace(-size / 2, size / 2, count + 1)
    x, y = np.meshgrid(xs, ys)
    z = terrain(scene, x, y).astype(np.float64)
    spawn_z = float(terrain(scene, 0.0, 0.0))
    z -= spawn_z
    dzdy, dzdx = np.gradient(z, ys, xs)
    normals = np.stack((-dzdx, -dzdy, np.ones_like(z)), axis=-1)
    normals /= np.linalg.norm(normals, axis=-1, keepdims=True)
    cx, cy = np.meshgrid((xs[:-1] + xs[1:]) / 2, (ys[:-1] + ys[1:]) / 2)
    center_z = terrain(scene, cx, cy) - spawn_z
    labels = material_ids(scene, cx, cy, center_z)

    # One runnable geometry check: every cell has a class and each scene has its intended obstacle.
    assert labels.shape == (count, count)
    if scene == "ramp":
        # Check the actual sampled center route, including background relief.
        route = z[count // 2]
        route_slope = np.degrees(np.arctan(np.abs(np.diff(route) / np.diff(xs))))
        assert abs(float(route_slope.max()) - RAMP_SLOPE_DEG) < 0.2
    elif scene == "curve":
        assert terrain(scene, 1.5, 4.0) - spawn_z < -0.15
    else:
        assert terrain(scene, -6.0, 7.0) - spawn_z < -0.15

    folder = PACKAGE / "models" / "terrain_v2" / scene
    folder.mkdir(parents=True, exist_ok=True)
    variants = rock_variants()
    for variant, geometry in enumerate(variants):
        write_rocks(folder.parent / f'boulder_{variant:02d}.obj', [geometry])
    # Compatibility alias for older resource readers; new worlds select variants.
    write_rocks(folder.parent/'boulder.obj',[variants[0]])
    models = []
    counts = {}
    mesh_labels = labels.copy()
    patch_info = None
    if scene == "survey":
        x0, x1 = PATCH_CENTER[0] - PATCH_SIZE / 2, PATCH_CENTER[0] + PATCH_SIZE / 2
        y0, y1 = PATCH_CENTER[1] - PATCH_SIZE / 2, PATCH_CENTER[1] + PATCH_SIZE / 2
        hole = (cx >= x0) & (cx < x1) & (cy >= y0) & (cy < y1)
        assert hole.sum() == 16 * 16 and np.all(labels[hole] == 0)
        mesh_labels[hole] = 255
        patch_x, patch_y, patch_z, residual = measured_patch(spawn_z)
        py_d, px_d = np.gradient(patch_z, patch_y, patch_x)
        patch_normals = np.stack((-px_d, -py_d, np.ones_like(patch_z)), axis=-1)
        patch_normals /= np.linalg.norm(patch_normals, axis=-1, keepdims=True)
        patch_labels = np.zeros((96, 96), dtype=np.uint8)
        patch_path = folder / "ce4_crater_patch.obj"
        assert write_obj(patch_path, patch_x, patch_y, patch_z, patch_normals,
                         patch_labels, 0) == 96 * 96
        models.append(model_xml(scene, "ce4_crater_patch",
                                {**CLASSES["regolith_passable"], "texture": "regolith_passable"}))
        vertex_hole = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        patch_cols = np.rint((x[vertex_hole] - x0) / PATCH_STEP).astype(int)
        patch_rows = np.rint((y[vertex_hole] - y0) / PATCH_STEP).astype(int)
        z[vertex_hole] = patch_z[patch_rows, patch_cols]
        patch_info = {
            "source_doi": "10.5281/zenodo.3732816",
            "source_paper_doi": "10.1029/2020GL087361",
            "source_mat_md5": "39a21d550aa9376c2497b26324bb76b9",
            "processed_npz_sha256": file_hash(PATCH_SOURCE),
            "center_m": PATCH_CENTER, "size_m": PATCH_SIZE,
            "source_step_m": 0.005, "mesh_step_m": PATCH_STEP,
            "coarse_cells_replaced": int(hole.sum()),
            "patch_cells": int(patch_labels.size),
            "residual_range_m": [round(float(residual.min()), 4),
                                 round(float(residual.max()), 4)],
            "edge_feather_m": 0.35,
            "transform": "north-up crop, Gaussian low-pass, 5 cm sampling, border-plane removal, edge feather; no horizontal or vertical scaling",
        }
        # Replace the synthetic mound/background with measured geometry inside.
        with np.load(BACKGROUND_SOURCE) as source:
            local_x, local_y, measured = source["x"], source["y"], source["z"]
            assert source["source_md5"].item() == "985ec0b225279df836cba87a5ecd306b"
            source_datum = float(source["removed_height_datum_m"])
        assert measured.shape == (121, 121) and np.all(np.isfinite(measured))
        assert abs(local_x[1] - local_x[0] - PATCH_STEP) < 1e-9
        background_x, background_y = BACKGROUND_CENTER[0] + local_x, BACKGROUND_CENTER[1] + local_y
        bx_grid, by_grid = np.meshgrid(background_x, background_y)
        lx, ly = np.meshgrid(local_x, local_y)
        distance = np.minimum(BACKGROUND_SIZE / 2 - np.abs(lx), BACKGROUND_SIZE / 2 - np.abs(ly))
        weight = smoothstep(distance / 0.35)
        # Blend to the actual coarse mesh edges, not the continuous formula.
        synthetic = np.array([np.interp(background_x, xs, row) for row in z])
        synthetic = np.array([np.interp(background_y, ys, col) for col in synthetic.T]).T
        background_z = (1 - weight) * synthetic + weight * measured
        x0, x1, y0, y1 = background_x[0], background_x[-1], background_y[0], background_y[-1]
        assert all(np.any(np.isclose(xs, v, atol=1e-9, rtol=0)) for v in (x0, x1))
        assert all(np.any(np.isclose(ys, v, atol=1e-9, rtol=0)) for v in (y0, y1))
        background_hole = (cx >= x0) & (cx < x1) & (cy >= y0) & (cy < y1)
        assert background_hole.sum() == 20 * 20 and not np.any(hole & background_hole)
        mesh_labels[background_hole] = 255
        bg_dy, bg_dx = np.gradient(background_z, background_y, background_x)
        bg_normals = np.stack((-bg_dx, -bg_dy, np.ones_like(background_z)), axis=-1)
        bg_normals /= np.linalg.norm(bg_normals, axis=-1, keepdims=True)
        bg_cx, bg_cy = np.meshgrid((background_x[:-1] + background_x[1:]) / 2,
                                  (background_y[:-1] + background_y[1:]) / 2)
        bg_cz = (background_z[:-1, :-1] + background_z[1:, 1:]) / 2
        bg_labels = material_ids(scene, bg_cx, bg_cy, bg_cz)
        background_counts = {}
        for name, config in CLASSES.items():
            bg_name = f"ce4_background_{name}"
            bg_path = folder / f"{bg_name}.obj"
            cells = write_obj(bg_path, background_x, background_y, background_z,
                              bg_normals, bg_labels, config["id"])
            if cells:
                models.append(model_xml(scene, bg_name, {**config, "texture": name}))
                background_counts[name] = cells
            elif bg_path.exists():
                bg_path.unlink()
        assert sum(background_counts.values()) == 120 * 120
        vertices = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        cols = np.rint((x[vertices] - x0) / PATCH_STEP).astype(int)
        rows = np.rint((y[vertices] - y0) / PATCH_STEP).astype(int)
        z[vertices] = background_z[rows, cols]
        center_cols = np.rint((cx[background_hole] - x0) / PATCH_STEP).astype(int)
        center_rows = np.rint((cy[background_hole] - y0) / PATCH_STEP).astype(int)
        labels[background_hole] = material_ids(scene, cx[background_hole], cy[background_hole],
                                              background_z[center_rows, center_cols])
        background_info = {
            "source_doi": "10.5281/zenodo.4971121", "source_paper_doi": "10.1029/2021GL094931",
            "source_tif_md5": "985ec0b225279df836cba87a5ecd306b",
            "processed_npz_sha256": file_hash(BACKGROUND_SOURCE),
            "source_crop_row0_col0": [2575, 1261],
            "center_m": BACKGROUND_CENTER, "size_m": BACKGROUND_SIZE,
            "mesh_step_m": PATCH_STEP, "source_step_m": 0.005,
            "coarse_cells_replaced": int(background_hole.sum()),
            "patch_cells_by_class": background_counts,
            "removed_height_datum_m": source_datum, "edge_feather_m": 0.35,
            "transform": "north-up valid crop, Gaussian low-pass, 5 cm sampling, center-height translation, edge blend; no plane removal or scaling",
            "scope": "measured interior; placement, boundary and labels engineered; not millimeter roughness validation",
        }
    for name, config in CLASSES.items():
        path = folder / f"{name}.obj"
        cells = write_obj(path, xs, ys, z, normals, mesh_labels, config["id"])
        if cells:
            models.append(model_xml(scene, name, config))
            counts[name] = cells
        elif path.exists():
            path.unlink()
    assert sum(counts.values()) == count * count - (16 * 16 + 20 * 20 if scene == "survey" else 0)

    boulders = []
    for index, (bx, by, rx, ry, rz) in enumerate(BOULDERS[scene], 1):
        variant = (index-1+list(SCENES).index(scene)*2) % 6
        ground_z = sample_ground(xs,ys,z,bx,by)
        if scene == "survey" and background_x[0] <= bx <= background_x[-1] and background_y[0] <= by <= background_y[-1]:
            # Current boulder center lies exactly on a fine-grid node.
            col = int(round((bx - background_x[0]) / PATCH_STEP))
            row = int(round((by - background_y[0]) / PATCH_STEP))
            assert abs(background_x[col] - bx) < 1e-9 and abs(background_y[row] - by) < 1e-9
            ground_z = float(background_z[row, col])
        if scene == "curve" and index == 1:
            # Use the written collision mesh, not an assumed sphere radius.
            mesh_top = max(float(line.split()[3]) for line in
                           (folder.parent / f"boulder_{variant:02d}.obj").read_text().splitlines()
                           if line.startswith("v "))
            rz = CURVE_REFERENCE_ROCK_HEIGHT / (0.65 + mesh_top)
            # This reference center lies on a mesh row: use the actual linear
            # collision surface there, not the unsampled analytic crater.
            row = int(np.searchsorted(ys, by))
            assert abs(ys[row] - by) < 1e-9
            ground_z = float(np.interp(bx, xs, z[row]))
        bz = ground_z + 0.65 * rz
        models.append(boulder_xml(index, bx, by, bz, rx, ry, rz, variant))
        boulders.append({"name": f"boulder_{index:02d}", "position_m": [bx, by, round(bz, 4)],
                         "radii_m": [rx, ry, rz], "variant":variant,
                         "class_id": CLASSES["rock_boulder"]["id"]})

    components, pebble_info = pebbles(scene,xs,ys,z,labels,variants,boulders)
    if components:
        write_rocks(folder / 'surface_pebbles.obj',components)
        models.append(model_xml(scene,'surface_pebbles',
                                {**CLASSES['rock_boulder'],'appearance':'rock_boulder'}))
    atlas_info = bake_scene(scene,labels,size,PACKAGE / 'materials/textures')

    mask_path = folder / "class_mask.png"
    Image.fromarray(np.flipud(labels), mode="L").save(mask_path)
    height_path = folder / "height_grid.npz"
    np.savez_compressed(height_path, x=xs, y=ys, z=z)
    world_path = PACKAGE / "worlds" / f"mooncar_{scene}_v2.sdf"
    world_path.write_text(world_xml(scene, models), encoding="utf-8")
    height_min = min(float(z.min()), float(patch_z.min()) if patch_info else float(z.min()))
    height_max = max(float(z.max()), float(patch_z.max()) if patch_info else float(z.max()))
    if scene == "survey":
        height_min = min(height_min, float(background_z.min()))
        height_max = max(height_max, float(background_z.max()))
    boulder_area_fraction = sum(math.pi * b[2] * b[3] for b in BOULDERS[scene]) / size**2
    manifest = {
        "physics_config": {"backend": PHYSICS_BACKEND, "step_seconds": PHYSICS_STEP,
                           "engine": PHYSICS_ENGINES[PHYSICS_BACKEND],
                           "scope": "numerical runtime configuration, not measured soil properties"},
        "scene": scene, "world": world_path.name, "size_m": size, "grid_step_m": step,
        "spawn_ground_z_m": 0.0, "height_min_m": round(height_min, 4),
        "height_max_m": round(height_max, 4), "cell_count_by_class": counts,
        "class_parameters": {name: {"id": c["id"], "mu": c["mu"], "mu2": c["mu2"]}
                             for name, c in CLASSES.items()},
        "boulders": boulders,
        "detail_version": "terrain-v2-detail-v1",
        "detail_seed": SEED,
        "surface_atlas": atlas_info,
        "pebbles": pebble_info,
        "rock_variant_sha256": {f'boulder_{i:02d}.obj':file_hash(folder.parent/f'boulder_{i:02d}.obj') for i in range(6)},
        "detail_generator_sha256": file_hash(SOURCE_PACKAGE/'scripts/terrain_scene_detail.py'),
        "material_generator_sha256": file_hash(SOURCE_PACKAGE/'scripts/generate_terrain_materials.py'),
        "boulder_projected_area_fraction": round(boulder_area_fraction, 7),
        "measured_patch": patch_info,
        "generator_sha256": file_hash(Path(__file__)),
        "contact_parameter_status": "engineering assumptions; not literature-calibrated mu/mu2",
        "labels": "class_mask.png is row-major from north to south; x increases east, y increases north",
        "world_sha256": file_hash(world_path),
        "mesh_sha256": {path.name: file_hash(path) for path in sorted(folder.glob("*.obj"))},
        "boulder_mesh_sha256": file_hash(folder.parent / "boulder.obj"),
        "texture_sha256": {path.name: file_hash(path) for path in sorted((PACKAGE / "materials" / "textures").glob("*.png"))
                           if path.name.startswith(scene+'_surface_') or
                           any(path.name.startswith(name+'_') for name in CLASSES)},
    }
    if scene == "ramp":
        manifest["geometry_reference"] = {
            "doi": "10.1038/s41467-024-46233-8", "page": 3,
            "route_reference_slope_deg": RAMP_SLOPE_DEG,
            "derived_ramp_height_m": RAMP_HEIGHT,
            "sampled_center_route_max_slope_deg": float(route_slope.max()),
            "sampled_surface_max_gradient_deg": float(np.degrees(np.arctan(np.hypot(dzdx, dzdy))).max()),
            "scope": "reference slope sets longitudinal ramp component; engineered shape and placement, not a route reconstruction or global slope cap",
        }
    elif scene == "survey":
        manifest["measured_background_patch"] = background_info
        manifest["geometry_reference"] = {
            "doi": "10.1029/2021GL094931", "pages": [3, 4], "region": "R3",
            "crater_diameter_m": 2 * SURVEY_CRATER_RADIUS,
            "crater_depth_diameter_ratio": SURVEY_CRATER_DEPTH_RATIO,
            "scope": "approximate observed degraded-crater scale; synthetic profile and placement, separate from measured fresh-crater patch",
        }
    if scene == "curve":
        manifest["reference_rock"] = {
            "name": "boulder_01", "doi": "10.1038/s41467-024-46233-8",
            "page": 7, "exposed_height_m": CURVE_REFERENCE_ROCK_HEIGHT,
            "datum": "top collision vertex relative to ground at rock center",
            "derived_vertical_scale_m": boulders[0]["radii_m"][2],
            "scope": "approximate observed height of one Longji target; XY shape, burial fraction and placement remain engineered",
        }
    if scene == "survey":
        k, cutoff = 0.0021, 0.25
        model_fraction = k * math.exp(-(0.5648 + 0.01258 / k) * cutoff)
        visible_fraction = sum(math.pi * rx * ry for _, _, rx, ry, _ in BOULDERS[scene]
                               if 2 * math.sqrt(rx * ry) > cutoff) / size**2
        assert abs(visible_fraction / model_fraction - 1) < 0.10
        manifest["resolved_rock_area_check"] = {
            "source": "Wu et al. 2021, Equation 6, k=0.21%",
            "diameter_cutoff_m": cutoff,
            "model_fraction_above_cutoff": round(model_fraction, 7),
            "explicit_boulder_proxy_fraction_above_cutoff": round(visible_fraction, 7),
            "scope": "large rocks are one engineered sample; new sub-0.10 m detail is not a measured abundance model",
        }
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"{scene}: {size:g} m, z [{height_min:.3f}, {height_max:.3f}] m, {counts}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root',type=Path,default=SOURCE_PACKAGE,
                        help='Gazebo package resource root; measured inputs always read from source package')
    parser.add_argument('--physics-backend',choices=PHYSICS_ENGINES,default=PHYSICS_BACKEND)
    parser.add_argument('--physics-step',type=float,choices=(.002,.001,.0005),default=PHYSICS_STEP)
    args = parser.parse_args()
    PHYSICS_BACKEND, PHYSICS_STEP = args.physics_backend, args.physics_step
    PACKAGE = args.output_root.resolve()
    required = [PACKAGE/'materials/textures'/f'{name}_{kind}.png'
                for name in CLASSES for kind in ('albedo','normal','roughness')]
    if not all(path.is_file() for path in required):
        parser.error('Generate tile materials first with generate_terrain_materials.py --out <output-root>/materials')
    (PACKAGE/'worlds').mkdir(parents=True,exist_ok=True)
    for scene_name in SCENES:
        build(scene_name)
