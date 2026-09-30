#!/usr/bin/env python3
"""Build three versioned lunar terrain worlds from one continuous surface per scene.

Visual and collision meshes use the same vertices. Material classes split the
surface into separate meshes so each class can also have a contact coefficient.
The survey world includes one measured DTM patch; remaining geometry and all
contact coefficients are simulation hypotheses.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = {"ramp": (16.0, 0.25), "curve": (16.0, 0.25), "survey": (30.0, 0.30)}
CURVE_CRATERS = ((1.5, 4.0, 3.05), (5.5, -5.5, 5.6))
PATCH_CENTER = (4.8, 3.6)
PATCH_SIZE = 4.8
PATCH_STEP = 0.05
PATCH_SOURCE = PACKAGE / "data/terrain_sources/ce4_fresh_crater_5cm.npz"
BOULDERS = {
    "ramp": [(-2.8, 3.8, 0.48, 0.40, 0.30), (4.3, -4.0, 0.62, 0.49, 0.38)],
    "curve": [(-0.7, 6.0, 0.55, 0.42, 0.31), (7.0, -0.1, 0.60, 0.50, 0.36)],
    "survey": [(-3.2, 4.5, 0.215, 0.215, 0.150), (1.7, 7.0, 0.170, 0.170, 0.120),
               (7.2, 3.0, 0.135, 0.135, 0.100), (-7.4, -1.5, 0.130, 0.130, 0.100),
               (4.0, -7.4, 0.120, 0.120, 0.090)],
}
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
    distance = np.hypot(x, y)
    fade = smoothstep((distance - 0.8) / 1.2)
    return fade * (0.013 * np.sin(0.8 * x + 0.3 * y) * np.sin(0.7 * y)
                   + 0.006 * np.sin(1.9 * x - 1.3 * y))


def crater(x, y, cx, cy, radius, depth):
    r = np.hypot(x - cx, y - cy)
    bowl = -depth * np.exp(-((r / (0.62 * radius)) ** 4))
    rim = 0.45 * depth * np.exp(-(((r - radius) / (0.22 * radius)) ** 2))
    return bowl + rim


def terrain(scene, x, y):
    z = base_relief(x, y)
    if scene == "ramp":
        ramp = smoothstep((x - 1.0) / 6.0)
        side = smoothstep((7.5 - np.abs(y)) / 6.0)
        return z + 0.42 * ramp * side
    if scene == "curve":
        for cx, cy, radius in CURVE_CRATERS:
            # The 6.1 m and 11.2 m diameters are observed near Yutu-2's route;
            # their local depth here follows the separate, typical d/D=0.061 study.
            z += crater(x, y, cx, cy, radius, 0.061 * (2 * radius) / 1.45)
        return z
    if scene == "survey":
        z += crater(x, y, -6.0, 7.0, 2.6, 0.061 * 5.2 / 1.45)
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
        for cx, cy, radius in ((-6.0, 7.0, 2.6),):
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
                stream.write(f"vt {xs[i] / 1.2:.6f} {ys[j] / 1.2:.6f}\n")
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
    tex = f"model://mooncar_gazebo/materials/textures/{config.get('texture', name)}"
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


def boulder_xml(index, x, y, z, rx, ry, rz):
    uri = "model://mooncar_gazebo/models/terrain_v2/boulder.obj"
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


def world_xml(scene, models):
    name = f"mooncar_{scene}_v2"
    return f"""<?xml version="1.0" encoding="utf-8"?>
<sdf version="1.10">
  <world name="{name}">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
      <engine><filename>gz-physics-bullet-featherstone-plugin</filename></engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <physics name="mooncar_v2_physics" type="ode">
      <max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor>
    </physics>
    <gravity>0 0 -1.62</gravity>
    <scene><ambient>0.42 0.43 0.46</ambient><background>0.11 0.12 0.15</background></scene>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.92 0.86 1</diffuse>
      <specular>0.20 0.20 0.20 1</specular>
      <direction>-0.45 -0.30 -0.84</direction>
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
        assert 0.38 < terrain(scene, 7.0, 0.0) - spawn_z < 0.44
    elif scene == "curve":
        assert terrain(scene, 1.5, 4.0) - spawn_z < -0.15
    else:
        assert terrain(scene, -6.0, 7.0) - spawn_z < -0.15

    folder = PACKAGE / "models" / "terrain_v2" / scene
    folder.mkdir(parents=True, exist_ok=True)
    write_boulder_obj(folder.parent / "boulder.obj")
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
    for name, config in CLASSES.items():
        path = folder / f"{name}.obj"
        cells = write_obj(path, xs, ys, z, normals, mesh_labels, config["id"])
        if cells:
            models.append(model_xml(scene, name, config))
            counts[name] = cells
        elif path.exists():
            path.unlink()
    assert sum(counts.values()) == count * count - (16 * 16 if scene == "survey" else 0)

    boulders = []
    for index, (bx, by, rx, ry, rz) in enumerate(BOULDERS[scene], 1):
        bz = float(terrain(scene, bx, by) - spawn_z) + 0.65 * rz
        models.append(boulder_xml(index, bx, by, bz, rx, ry, rz))
        boulders.append({"name": f"boulder_{index:02d}", "position_m": [bx, by, round(bz, 4)],
                         "radii_m": [rx, ry, rz], "class_id": CLASSES["rock_boulder"]["id"]})

    mask_path = folder / "class_mask.png"
    Image.fromarray(np.flipud(labels), mode="L").save(mask_path)
    height_path = folder / "height_grid.npz"
    np.savez_compressed(height_path, x=xs, y=ys, z=z)
    world_path = PACKAGE / "worlds" / f"mooncar_{scene}_v2.sdf"
    world_path.write_text(world_xml(scene, models), encoding="utf-8")
    height_min = min(float(z.min()), float(patch_z.min()) if patch_info else float(z.min()))
    height_max = max(float(z.max()), float(patch_z.max()) if patch_info else float(z.max()))
    boulder_area_fraction = sum(math.pi * b[2] * b[3] for b in BOULDERS[scene]) / size**2
    manifest = {
        "scene": scene, "world": world_path.name, "size_m": size, "grid_step_m": step,
        "spawn_ground_z_m": 0.0, "height_min_m": round(height_min, 4),
        "height_max_m": round(height_max, 4), "cell_count_by_class": counts,
        "class_parameters": {name: {"id": c["id"], "mu": c["mu"], "mu2": c["mu2"]}
                             for name, c in CLASSES.items()},
        "boulders": boulders,
        "boulder_projected_area_fraction": round(boulder_area_fraction, 7),
        "measured_patch": patch_info,
        "labels": "class_mask.png is row-major from north to south; x increases east, y increases north",
        "world_sha256": file_hash(world_path),
        "mesh_sha256": {path.name: file_hash(path) for path in sorted(folder.glob("*.obj"))},
        "boulder_mesh_sha256": file_hash(folder.parent / "boulder.obj"),
        "texture_sha256": {path.name: file_hash(path) for path in sorted((PACKAGE / "materials" / "textures").glob("*.png"))},
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
            "scope": "one engineered discrete sample; unresolved smaller rocks are not collision objects",
        }
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"{scene}: {size:g} m, z [{height_min:.3f}, {height_max:.3f}] m, {counts}")


if __name__ == "__main__":
    for scene_name in SCENES:
        build(scene_name)
