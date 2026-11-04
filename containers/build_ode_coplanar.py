#!/usr/bin/env python3
"""Build a project-local, test-only ODE coplanar cylinder contact repair."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import time
from urllib.request import urlopen

VERSION = '0.16.2'
URL = 'https://archive.ubuntu.com/ubuntu/pool/universe/o/ode/ode_0.16.2.orig.tar.gz'
ARCHIVE_SHA = 'b26aebdcb015e2d89720ef48e0cb2e8a3ca77915f89d853893e7cc861f810f22'

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def build(output, archive=None):
    output=Path(output)
    if output.exists():
        raise FileExistsError('Refusing to overwrite a previous build')
    output.mkdir(parents=True)
    archive=Path(archive) if archive else output/'ode.tar.gz'
    if not archive.exists(): archive.write_bytes(urlopen(URL, timeout=60).read())
    if digest(archive)!=ARCHIVE_SHA: raise RuntimeError('ODE source checksum mismatch')
    source=output/'source'; source.mkdir()
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            parts=Path(member.name).parts
            if '..' in parts or Path(member.name).is_absolute() or not (member.isfile() or member.isdir()):
                raise RuntimeError('Unsafe archive entry')
            target=source.joinpath(*parts[1:])
            if member.isdir(): target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(tar.extractfile(member).read())
                target.chmod(member.mode & 0o777)
    target=source/'ode/src/collision_cylinder_trimesh.cpp'
    original=target.read_text()
    (output/'collision_cylinder_trimesh-original.cpp').write_text(original)
    needle='int dCollideCylinderTrimesh(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)'
    include='// MoonCar: fully supported coplanar patches only.\n#include "mooncar_coplanar_contact.h"\n\n'
    original=original.replace(needle, include+needle, 1)
    hook='''        int planarContacts=0;
        if (mooncar_coplanar::collide(Cylinder, Trimesh, Triangles, TriCount,
                                      flags, contact, skip, planarContacts))
            return planarContacts;

'''
    needle='        // allocate buffer for local contacts on stack'
    if original.count(needle)!=1: raise RuntimeError('Unexpected source layout')
    target.write_text(original.replace(needle, hook+needle, 1))
    header=Path(__file__).with_name('r5_coplanar_contact')/'coplanar_contact.h'
    shutil.copy2(header, source/'ode/src/mooncar_coplanar_contact.h')
    started=time.monotonic()
    def execute(argv, name, cwd=None):
        with (output/name).open('w') as log:
            subprocess.run(argv, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                           check=True, timeout=max(1, 2700-(time.monotonic()-started)))
    # Match the Ubuntu package's autotools / double precision / system libccd
    # configuration. Do not reuse the earlier observational CMake binary.
    execute(['bash', 'bootstrap'], 'bootstrap.log', source)
    # ODE's VPATH include order prefers source/include over build/include.
    # The archive ships a single-precision header: configure in source, as the
    # Debian package does, so the generated double header is actually consumed.
    build_dir=source
    execute([str(source.resolve()/'configure'), '--enable-double-precision',
             '--with-libccd=system', '--enable-libccd', '--enable-shared',
             '--enable-static', '--disable-demos', '--disable-tests'], 'configure.log', build_dir)
    execute(['make', '-j2'], 'compile.log', build_dir)
    runtime=output/'runtime'; runtime.mkdir()
    library=runtime/'libode.so.8.0.2'
    shutil.copy2(build_dir/'ode/src/.libs/libode.so.8.0.2', library)
    (runtime/'libode.so.8').symlink_to(library.name)
    loaded=ctypes.CDLL(str(library.resolve()))
    loaded.dGetConfiguration.restype=ctypes.c_char_p
    configuration=loaded.dGetConfiguration().decode()
    if 'ODE_double_precision' not in configuration or 'ODE_single_precision' in configuration:
        raise RuntimeError('Actual ODE precision does not match the installed ABI')
    manifest=dict(version=VERSION, source_url=URL, source_sha256=ARCHIVE_SHA,
        original_sha256=digest(output/'collision_cylinder_trimesh-original.cpp'),
        patched_sha256=digest(target), header_sha256=digest(header),
        library_sha256=digest(library), build_seconds=time.monotonic()-started,
        actual_configuration=configuration,
        scope='test-only; full coplanar support; no formal library/default changes')
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest), flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--archive')
    args=parser.parse_args()
    build(args.output, args.archive)
