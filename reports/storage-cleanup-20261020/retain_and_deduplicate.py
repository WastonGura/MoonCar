"""One-time scoped retention audit. Only identical frozen data shares storage.

Does not remove source, reports, raw observations, or baseline snapshots.
Hard-linked evidence is immutable: copy a file before editing a retained record.
"""
import collections
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

ROOT = Path(r'D:\Documents\Projects\Repositories\MoonCar-mapping').resolve()
AUDIT = ROOT / 'reports' / 'storage-cleanup-20261020'
DATA = ROOT / 'reports' / 'evidence'
ALLOWED = {'.csv', '.jsonl', '.npz', '.db3', '.png', '.zip', '.gz', '.obj', '.ply', '.pdf', '.tif', '.mat'}


def write(name, value):
    (AUDIT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def files(root):
    for dp, dirs, names in os.walk(root, followlinks=False):
        dirs[:] = [n for n in dirs if not Path(dp, n).is_symlink()
                   and not os.path.isjunction(Path(dp, n))
                   and n not in {'node_modules', '__pycache__', '.git', '.pytest_cache'}]
        for name in names:
            path = Path(dp, name)
            if not path.is_symlink():
                yield path


def protect():
    paths = []
    for name in ['ros2_ws', 'scripts', 'containers', 'web', 'firmware', 'docs', 'assets', '.github']:
        paths.extend(files(ROOT / name))
    paths.extend(p for p in ROOT.iterdir() if p.is_file())
    paths.extend(p for p in (ROOT / 'reports').iterdir() if p.is_file())
    # Freeze all historical integrity manifests as well as source and reports.
    paths.extend(DATA.rglob('verification.json'))
    return {str(p.relative_to(ROOT)): digest(p) for p in paths}


def main():
    assert DATA.resolve().is_relative_to(ROOT)
    protected = protect()
    write('protected-sha256.json', protected)
    sizes = collections.defaultdict(list)
    total = count = 0
    inaccessible = []
    for p in files(DATA):
        try:
            st = p.stat()
        except OSError as error:
            inaccessible.append({'path': str(p.relative_to(ROOT)), 'error': str(error)})
            continue
        total += st.st_size
        count += 1
        if p.suffix.lower() in ALLOWED and st.st_size >= 1024**2:
            sizes[st.st_size].append(p)
    print(json.dumps({'phase': 'inventoried', 'evidence_files': count, 'logical_bytes': total,
                      'same_size_candidate_files': sum(len(v) for v in sizes.values() if len(v) > 1)}), flush=True)
    samples = collections.defaultdict(list)
    for size, paths in sizes.items():
        if len(paths) < 2:
            continue
        for p in paths:
            h = hashlib.sha256()
            with p.open('rb') as f:
                for offset in [0, max(0, size//2 - 32768), max(0, size - 65536)]:
                    f.seek(offset)
                    h.update(f.read(65536))
            samples[(size, h.digest())].append(p)
    matched = collections.defaultdict(list)
    last = time.monotonic()
    hashed = 0
    for (size, _), paths in samples.items():
        if len(paths) < 2:
            continue
        for p in paths:
            matched[(size, digest(p))].append(p)
            hashed += size
            if time.monotonic() - last > 25:
                print(json.dumps({'phase': 'full_sha256_comparison', 'bytes_read': hashed}), flush=True)
                last = time.monotonic()
    changes = []
    saved = 0
    for (size, sha), paths in matched.items():
        if len(paths) < 2:
            continue
        paths.sort(key=lambda p: (len(str(p)), str(p)))
        canonical = paths[0]
        for duplicate in paths[1:]:
            assert canonical.resolve().is_relative_to(DATA)
            assert duplicate.resolve().is_relative_to(DATA)
            if os.path.samefile(canonical, duplicate):
                continue
            # Full digest was checked for both, not merely size or sampled bytes.
            temporary = duplicate.with_name('.mooncar-storage-link-' + uuid.uuid4().hex + '.tmp')
            try:
                os.link(canonical, temporary)
                os.replace(temporary, duplicate)
            finally:
                if temporary.exists():
                    temporary.unlink()
            assert os.path.samefile(canonical, duplicate)
            saved += size
            changes.append({'retained_path': str(duplicate.relative_to(ROOT)),
                            'canonical_path': str(canonical.relative_to(ROOT)),
                            'sha256': sha, 'bytes': size})
    result = {'method': 'full-SHA256-verified NTFS hardlinks within frozen evidence only',
              'logical_paths_preserved': True, 'raw_data_removed': False,
              'immutable_record_rule': 'Copy before editing any linked evidence file.',
              'evidence_logical_bytes_before': total, 'evidence_files_before': count,
              'inaccessible_preserved': inaccessible,
              'duplicate_files_merged': len(changes), 'duplicate_logical_bytes_removed': saved,
              'links': changes}
    write('deduplication.json', result)
    current = protect()
    assert current == protected, 'Protected source or report digest changed'
    write('protected-verification.json', {'files': len(protected), 'all_unchanged': True})
    print(json.dumps({k:v for k,v in result.items() if k != 'links'}), flush=True)


if __name__ == '__main__':
    main()
