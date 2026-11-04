#!/usr/bin/env python3
"""Build a pinned, project-local DART mesh-contact import repair (no apt install)."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import time
from urllib.request import urlopen

VERSION = '7.8.0'
ARCHIVE_SHA = '0d95b1dd3ea8c6f558535efe5fab6ecf5af947902a4367982e6fabd586752266'
URL = 'https://codeload.github.com/gazebosim/gz-physics/tar.gz/refs/tags/gz-physics7_7.8.0'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(output, archive=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    source = output/'source'
    if source.exists():
        raise RuntimeError('Refusing to overwrite a prior build')
    archive = Path(archive) if archive else output/'source.tar.gz'
    if not archive.exists():
        archive.write_bytes(urlopen(URL, timeout=60).read())
    if digest(archive) != ARCHIVE_SHA:
        raise RuntimeError('Pinned source archive checksum mismatch')
    source.mkdir()
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            parts = Path(member.name).parts
            if '..' in parts or Path(member.name).is_absolute() or not (member.isfile() or member.isdir()):
                raise RuntimeError('Unsafe source archive entry')
            path = source.joinpath(*parts[1:])
            if member.isdir():
                path.mkdir(parents=True, exist_ok=True)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(tar.extractfile(member).read())
    target = source/'dartsim/src/SDFFeatures.cc'
    original = target.read_text()
    start = original.index('static ShapeAndTransform ConstructMesh(')
    end = original.index('\n/////////////////////////////////////////////////', start)
    replacement = '''static ShapeAndTransform ConstructMesh(const ::sdf::Mesh &_mesh)
{
  // MoonCar: construct the explicit resolved mesh in ConstructSdfCollision,
  // so the existing surface/friction import reaches its DynamicsAspect.
  // Unsupported resources fail visibly instead of silently ignoring friction.
  if (!_mesh.Submesh().empty())
    throw std::runtime_error("MoonCar DART: submesh contact import unsupported");
  const auto *input = common::MeshManager::Instance()->Load(_mesh.Uri());
  if (nullptr == input)
    throw std::runtime_error("MoonCar DART: unresolved collision mesh " + _mesh.Uri());
  return {std::make_shared<CustomMeshShape>(
      *input, math::eigen3::convert(_mesh.Scale()))};
}
'''
    (output/'SDFFeatures-original.cc').write_text(original)
    target.write_text('#include <stdexcept>\n'+original[:start]+replacement+original[end:])
    started = time.monotonic()
    def execute(argv, name):
        with (output/name).open('w') as log:
            subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT, check=True,
                           timeout=max(1, 1800-(time.monotonic()-started)))
    execute(['cmake', '-S', str(source), '-B', str(output/'build'),
             '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF'], 'configure.log')
    execute(['cmake', '--build', str(output/'build'), '--target',
             'gz-physics7-dartsim-plugin', '-j2'], 'compile.log')
    library = output/'libmooncar-physics7-dartsim.so'
    shutil.copy2(output/'build/lib/libgz-physics7-dartsim-plugin.so.7.8.0', library)
    manifest = dict(gz_physics_version=VERSION, source_url=URL, source_sha256=ARCHIVE_SHA,
                    original_sha256=hashlib.sha256(original.encode()).hexdigest(),
                    patched_sha256=digest(target), library_sha256=digest(library),
                    build_seconds=time.monotonic()-started,
                    role='mesh contact import software repair; no physical calibration')
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--archive')
    args = parser.parse_args()
    build(args.output, args.archive)
