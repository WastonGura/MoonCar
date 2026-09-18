"""Observed XYZ archive and conservative single-surface elevation map (no ROS).

Coordinates are metres in one gravity-aligned session frame. Pose corrections
must be applied before fusion; this module does not implement SLAM loop closure.
"""
import json
import io
import math
import os
from pathlib import Path
import tempfile
import zipfile

import numpy as np

from terrain_geometry import transform_points


def open_archive(path):
    # Capture one atomic file version before the expensive point fusion starts.
    with open(path, 'rb') as handle:
        stat = os.fstat(handle.fileno())
        if stat.st_size > 150000000:
            raise ValueError('Archive exceeds supported capacity')
        blob = handle.read(150000001)
    if len(blob) > 150000000:
        raise ValueError('Archive exceeds supported capacity')
    with zipfile.ZipFile(io.BytesIO(blob)) as archive:
        if len(archive.infolist()) not in (6, 8) or sum(i.file_size for i in archive.infolist()) > 150000000:
            raise ValueError('Archive exceeds supported capacity')
    return np.load(io.BytesIO(blob), allow_pickle=False), stat.st_mtime_ns


class TerrainMap:
    def __init__(self, resolution=0.25, voxel_size=0.05, frame='map', source='observations'):
        if not all(math.isfinite(v) and v > 0 for v in (resolution, voxel_size)):
            raise ValueError('Map resolutions must be finite and positive')
        if voxel_size > resolution / 2:
            raise ValueError('Voxels must be at most half the elevation cell size')
        if not isinstance(frame, str) or not frame or not isinstance(source, str) or not source:
            raise ValueError('Frame and source must be named')
        self.resolution, self.voxel_size = resolution, voxel_size
        self.frame, self.source = frame, source
        self.frames = []
        self.robot_poses = []
        self._voxels = {}
        self._views = {}
        self._sampling = {}
        self._cloud = None
        self._elevation_cache = {}

    def add_frame(self, points, stamp, translation=(0., 0., 0.), quaternion=(0., 0., 0., 1.),
                  robot_translation=None, robot_quaternion=None):
        """Store sensor-frame samples and their timestamped sensor-to-session pose.

        Capacity limits fail explicitly rather than silently dropping old terrain.
        Repeated timestamps are rejected to avoid duplicate-message confidence.
        """
        points = np.asarray(points, dtype=np.float64)
        translation, quaternion = np.asarray(translation, dtype=float), np.asarray(quaternion, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not 3 <= len(points) <= 100000:
            raise ValueError('Expected 3..100000 XYZ samples')
        if not np.isfinite(points).all() or np.max(np.abs(points)) > 10000:
            raise ValueError('Invalid XYZ samples')
        if (translation.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(translation).all()
                or np.any(np.abs(translation) > 10000) or not np.isfinite(quaternion).all()
                or np.linalg.norm(quaternion) < 1e-9):
            raise ValueError('Invalid pose')
        if not math.isfinite(stamp) or stamp < 0 or (self.frames and stamp <= self.frames[-1][0]):
            raise ValueError('Timestamps must increase within a session')
        robot = None
        if robot_translation is not None or robot_quaternion is not None:
            rt, rq = np.asarray(robot_translation, dtype=float), np.asarray(robot_quaternion, dtype=float)
            if (rt.shape != (3,) or rq.shape != (4,) or not np.isfinite(rt).all()
                    or not np.isfinite(rq).all() or np.linalg.norm(rq) < 1e-9 or np.any(np.abs(rt) > 10000)):
                raise ValueError('Invalid robot pose')
            robot = np.concatenate((rt, rq / np.linalg.norm(rq)))
        if len(self.frames) >= 5000 or sum(len(f[1]) for f in self.frames) + len(points) > 5000000:
            raise ValueError('Session capacity reached; save and start another session')
        # Fuse the same precision that is archived, including voxel-boundary cases.
        points = points.astype(np.float32)
        world = transform_points(points, translation, quaternion)
        if not np.isfinite(world).all() or np.max(np.abs(world)) > 10000:
            raise ValueError('Invalid transformed coordinates')
        keys, inverse = np.unique(np.floor(world / self.voxel_size).astype(np.int64),
                                  axis=0, return_inverse=True)
        sums = np.zeros((len(keys), 3))
        np.add.at(sums, inverse, world)
        counts = np.bincount(inverse)
        self._cloud = None
        for key, total, count in zip(keys, sums, counts):
            key = tuple(key)
            old_total, old_count = self._voxels.get(key, (np.zeros(3), 0))
            self._voxels[key] = (old_total + total, old_count + int(count))
        for cell in np.unique(np.floor(world[:, :2] / self.resolution).astype(np.int64), axis=0):
            key = tuple(cell)
            self._views[key] = self._views.get(key, 0) + 1
        # Spatial coverage and bearing diversity saturate; duplicate frames cannot inflate quality.
        bins=np.floor(world[:, :2]/(self.resolution/4)).astype(np.int64)
        for bx,by in np.unique(bins,axis=0):
            key=(int(bx//4),int(by//4));mask,views=self._sampling.get(key,(0,0))
            angle=math.atan2(translation[1]-(key[1]+.5)*self.resolution,translation[0]-(key[0]+.5)*self.resolution)
            bearing=int((angle+math.pi)/(math.pi/4))%8
            self._sampling[key]=(mask | (1 << int((by%4)*4+bx%4)),views | (1 << bearing))
        self.frames.append((float(stamp), points.astype(np.float32).copy(),
                            translation.copy(), quaternion.copy()))
        self.robot_poses.append(robot)

    def cloud(self):
        if self._cloud is None:
            self._cloud = np.asarray([total / n for _, (total, n) in sorted(self._voxels.items())],
                                     dtype=np.float64).reshape(-1, 3)
        return self._cloud.copy()

    def quality(self, with_surface=False):
        cells=[]
        for (ix,iy),(mask,views) in self._sampling.items():
            coverage=mask.bit_count()/16;directions=views.bit_count()
            cells.append(dict(ix=ix,iy=iy,coverage=coverage,view_directions=directions,
                              quality=round(coverage*(.7+.3*min(1,directions/3)),3)))
        if with_surface:
            fits={(c['ix'],c['iy']):c for c in self.elevation()}
            for c in cells:
                fit=fits[(c['ix'],c['iy'])]
                state=('valid' if fit['valid'] else 'too_few_voxels' if fit['points']<6
                       else 'insufficient_xy_spread' if fit['z'] is None
                       else 'steep_fit' if fit['slope']>=60 else 'nonplanar')
                c.update(fit_state=state,points=fit['points'],
                         bearing_mask=self._sampling[(c['ix'],c['iy'])][1],
                         z=fit['z'],gx=fit['gx'],gy=fit['gy'],slope=fit['slope'],roughness=fit['roughness'],valid=fit['valid'])
        return dict(resolution=self.resolution,cells=cells,frames=len(self.frames),
                    stamp=self.frames[-1][0] if self.frames else 0,
                    meaning='spatial sampling and bearing diversity; not accuracy or safety probability')

    def elevation(self):
        """Fit observed voxel centres; invalid cells retain support but no surface.

        A plane through a vertical wall or a scanline is not ground. High residual
        (including multiple surfaces) remains a blocked/invalid cell. No hole fill.
        """
        cloud = self.cloud().astype(float)
        if not len(cloud):
            return []
        keys, inverse = np.unique(np.floor(cloud[:, :2] / self.resolution).astype(int),
                                  axis=0, return_inverse=True)
        order = np.argsort(inverse, kind='stable')
        groups = np.split(order, np.cumsum(np.bincount(inverse))[:-1])
        result = []
        quality={(c['ix'],c['iy']):c for c in self.quality()['cells']}
        for key, indices in zip(keys, groups):
            sample = cloud[indices]
            cell_key = tuple(key)
            samples = sample.tobytes()
            cached = self._elevation_cache.get(cell_key)
            if cached and cached[0] == samples and cached[1]['observations'] == self._views[cell_key]:
                result.append(cached[1].copy())
                continue
            center = (key + .5) * self.resolution
            cell = dict(ix=int(key[0]), iy=int(key[1]), x=float(center[0]), y=float(center[1]),
                        z=None, gx=None, gy=None, slope=None, roughness=None,
                        support=0., observations=self._views[tuple(key)], points=len(sample), valid=False)
            cell.update(quality.get(tuple(key),{}))
            if len(sample) >= 6:
                design = np.column_stack((sample[:, :2] - center, np.ones(len(sample))))
                plane, _, rank, singular = np.linalg.lstsq(design, sample[:, 2], rcond=None)
                if rank == 3 and singular[-1] / math.sqrt(len(sample)) >= self.resolution * .05:
                    roughness = float(np.sqrt(np.mean((design @ plane - sample[:, 2]) ** 2)))
                    slope = math.degrees(math.atan(np.linalg.norm(plane[:2])))
                    cell.update(z=float(plane[2]), gx=float(plane[0]), gy=float(plane[1]),
                                slope=slope, roughness=roughness,
                                support=float(min(1., len(sample) / 16) * min(1., cell['observations'] / 3)),
                                valid=bool(slope < 60 and roughness < .10))
            self._elevation_cache[cell_key] = (samples, cell.copy())
            result.append(cell)
        return result

    def metadata(self):
        return dict(format='mooncar-observations-v2', frame=self.frame, source=self.source,
                    resolution=self.resolution, voxel_size=self.voxel_size,
                    frames=len(self.frames), robot_frame='base_footprint',
                    support_meaning='sampling support, not safety probability',
                    limitation='single surface elevation; pose errors accumulate; no loop-closure correction')

    def snapshot(self):
        robot_trajectory = [dict(stamp=f[0], position=p[:3].tolist(), quaternion=p[3:].tolist())
                            for f, p in zip(self.frames, self.robot_poses) if p is not None]
        return dict(**self.metadata(), points=self.cloud().tolist(), cells=self.elevation(),
                    trajectory=[dict(stamp=f[0], position=f[2].tolist()) for f in self.frames],
                    robot_trajectory=robot_trajectory,
                    robot=robot_trajectory[-1] if self.robot_poses and self.robot_poses[-1] is not None else None,
                    trajectory_frame='sensor origin', stamp=self.frames[-1][0] if self.frames else None)

    def save(self, path):
        """Atomic compressed archive; no pickle, world mesh or synthetic fill."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        offsets = np.cumsum([0] + [len(f[1]) for f in self.frames], dtype=np.int64)
        with tempfile.NamedTemporaryFile(dir=path.parent, suffix='.tmp', delete=False) as handle:
            temp = Path(handle.name)
            try:
                np.savez_compressed(handle, metadata=np.array(json.dumps(self.metadata())),
                                    points=np.concatenate([f[1] for f in self.frames]) if self.frames else np.empty((0, 3)),
                                    offsets=offsets, stamps=np.array([f[0] for f in self.frames]),
                                    translations=np.array([f[2] for f in self.frames]).reshape(-1, 3),
                                    quaternions=np.array([f[3] for f in self.frames]).reshape(-1, 4),
                                    robot_poses=np.array([p if p is not None else np.zeros(7) for p in self.robot_poses]).reshape(-1, 7),
                                    robot_valid=np.array([p is not None for p in self.robot_poses], dtype=bool))
                handle.flush()
                os.fsync(handle.fileno())
            except Exception:
                handle.close()
                temp.unlink(missing_ok=True)
                raise
        try:
            os.replace(temp, path)
        finally:
            temp.unlink(missing_ok=True)

    @classmethod
    def inspect(cls, path):
        data, modified = open_archive(path)
        with data:
            meta = json.loads(str(data['metadata']))
        if (meta.get('format') not in ('mooncar-observations-v1', 'mooncar-observations-v2')
                or type(meta.get('frames')) is not int or not 0 <= meta['frames'] <= 5000):
            raise ValueError('Invalid map metadata')
        cls(meta['resolution'], meta['voxel_size'], meta['frame'], meta['source'])
        return dict(**meta, archive_updated_ns=modified)

    @classmethod
    def load(cls, path, until=None):
        data, modified = open_archive(path)
        with data:
            meta = json.loads(str(data['metadata']))
            if meta.get('format') not in ('mooncar-observations-v1', 'mooncar-observations-v2'):
                raise ValueError('Unsupported map archive')
            points, offsets, stamps = data['points'], data['offsets'], data['stamps']
            translations, quaternions = data['translations'], data['quaternions']
            count = len(stamps)
            robot_poses = data['robot_poses'] if meta['format'].endswith('v2') else np.zeros((count, 7))
            robot_valid = data['robot_valid'] if meta['format'].endswith('v2') else np.zeros(count, dtype=bool)
            if (robot_poses.shape != (count, 7) or robot_valid.shape != (count,)
                    or robot_valid.dtype.kind != 'b' or not np.isfinite(robot_poses).all()
                    or np.any(np.abs(robot_poses[:, :3]) > 10000)
                    or np.any(np.linalg.norm(robot_poses[robot_valid, 3:], axis=1) < 1e-9)):
                raise ValueError('Invalid archived robot poses')
            if (count > 5000 or points.ndim != 2 or points.shape[1] != 3 or len(points) > 5000000
                    or offsets.dtype.kind not in 'iu' or offsets.shape != (count + 1,)
                    or offsets[0] != 0 or offsets[-1] != len(points) or np.any(np.diff(offsets) < 3)
                    or np.any(np.diff(offsets) > 100000)
                    or translations.shape != (count, 3) or quaternions.shape != (count, 4)
                    or stamps.shape != (count,) or meta.get('frames') != count):
                raise ValueError('Invalid archive arrays')
            if until is not None and (not isinstance(until, int) or not 0 <= until <= count):
                raise ValueError('Invalid replay frame count')
            result = cls(meta['resolution'], meta['voxel_size'], meta['frame'], meta['source'])
            result.archive_updated_ns = modified
            # Validate all frames even when replay displays a prefix.
            for i in range(count):
                if not np.isfinite(stamps[i]) or stamps[i] < 0 or (i and stamps[i] <= stamps[i - 1]):
                    raise ValueError('Invalid archive timestamps')
            if not np.isfinite(points).all() or not np.isfinite(translations).all() or not np.isfinite(quaternions).all():
                raise ValueError('Invalid archive values')
            if np.any(np.abs(points) > 10000) or np.any(np.abs(translations) > 10000):
                raise ValueError('Archive coordinates exceed supported range')
            if np.any(np.linalg.norm(quaternions, axis=1) < 1e-9):
                raise ValueError('Invalid archive quaternion')
            for i in range(count if until is None else until):
                result.add_frame(points[offsets[i]:offsets[i + 1]], float(stamps[i]), translations[i], quaternions[i],
                                 robot_poses[i, :3] if robot_valid[i] else None,
                                 robot_poses[i, 3:] if robot_valid[i] else None)
        return result

    def export_ply(self, path):
        cloud = self.cloud()
        with open(path, 'w', encoding='ascii', newline='\n') as handle:
            handle.write(f'ply\nformat ascii 1.0\ncomment {self.frame} observed voxel centroids\n'
                         f'element vertex {len(cloud)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
            np.savetxt(handle, cloud, fmt='%.6f')

    def export_dem(self, path, nodata=-9999.0):
        """Export standardized 2.5D DEM raster grid (ESRI ASCII Grid .asc) and JSON metadata."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        cells = [c for c in self.elevation() if c.get('valid') and c.get('z') is not None]
        if not cells:
            header = f"ncols        1\nnrows        1\nxllcorner    0.0000\nyllcorner    0.0000\ncellsize     {self.resolution:.4f}\nNODATA_value {nodata:.1f}\n{nodata:.1f}\n"
            path.write_text(header, encoding='ascii')
            meta = dict(ncols=1, nrows=1, xllcorner=0.0, yllcorner=0.0, cellsize=self.resolution,
                        nodata=nodata, valid_cells=0, total_cells=1, min_z=None, max_z=None, mean_z=None)
            Path(str(path).rsplit('.', 1)[0] + '.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
            return meta

        min_ix = min(c['ix'] for c in cells)
        max_ix = max(c['ix'] for c in cells)
        min_iy = min(c['iy'] for c in cells)
        max_iy = max(c['iy'] for c in cells)
        ncols = max_ix - min_ix + 1
        nrows = max_iy - min_iy + 1
        xllcorner = min_ix * self.resolution
        yllcorner = min_iy * self.resolution
        cellsize = self.resolution

        grid = np.full((nrows, ncols), nodata, dtype=float)
        for c in cells:
            row = max_iy - c['iy']
            col = c['ix'] - min_ix
            grid[row, col] = round(c['z'], 4)

        header = (f"ncols        {ncols}\n"
                  f"nrows        {nrows}\n"
                  f"xllcorner    {xllcorner:.4f}\n"
                  f"yllcorner    {yllcorner:.4f}\n"
                  f"cellsize     {cellsize:.4f}\n"
                  f"NODATA_value {nodata:.1f}\n")
        lines = [header]
        for row in grid:
            lines.append(' '.join(f'{v:.4f}' if v != nodata else str(int(nodata)) for v in row))
        path.write_text('\n'.join(lines) + '\n', encoding='ascii')

        valid_vals = grid[grid != nodata]
        meta = dict(ncols=int(ncols), nrows=int(nrows),
                    xllcorner=float(xllcorner), yllcorner=float(yllcorner),
                    cellsize=float(cellsize), nodata=float(nodata),
                    valid_cells=int(len(valid_vals)), total_cells=int(ncols * nrows),
                    min_z=float(np.min(valid_vals)) if len(valid_vals) else None,
                    max_z=float(np.max(valid_vals)) if len(valid_vals) else None,
                    mean_z=float(np.mean(valid_vals)) if len(valid_vals) else None)
        json_path = Path(str(path).rsplit('.', 1)[0] + '.json')
        json_path.write_text(json.dumps(meta, indent=2), encoding='utf-8')
        return meta

    def export_mesh(self, path):
        """Export continuous 3D triangle surface mesh in Wavefront OBJ format."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        valid_cells = {(c['ix'], c['iy']): c['z'] for c in self.elevation()
                       if c.get('valid') and c.get('z') is not None}
        if not valid_cells:
            path.write_text("# Empty mesh\n", encoding='ascii')
            return 0, 0

        vertices = []
        vertex_map = {}
        for idx, (key, z) in enumerate(sorted(valid_cells.items()), start=1):
            vertex_map[key] = idx
            wx = (key[0] + 0.5) * self.resolution
            wy = (key[1] + 0.5) * self.resolution
            vertices.append(f"v {wx:.4f} {wy:.4f} {z:.4f}")

        faces = []
        for (ix, iy) in valid_cells:
            v00 = vertex_map.get((ix, iy))
            v10 = vertex_map.get((ix + 1, iy))
            v01 = vertex_map.get((ix, iy + 1))
            v11 = vertex_map.get((ix + 1, iy + 1))
            if v00 and v10 and v11:
                faces.append(f"f {v00} {v10} {v11}")
            if v00 and v11 and v01:
                faces.append(f"f {v00} {v11} {v01}")

        content = [
            f"# MoonCar 3D Surface Mesh ({self.frame})",
            f"# Resolution: {self.resolution:.3f}m, Vertices: {len(vertices)}, Faces: {len(faces)}",
            *vertices,
            *faces,
            ""
        ]
        path.write_text('\n'.join(content), encoding='ascii')
        return len(vertices), len(faces)
