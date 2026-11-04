"""Lossless NTFS compression of retained large text records, with bounded workers.

No source tree, baseline configuration, filename or file contents are removed.
"""
from concurrent.futures import ThreadPoolExecutor, as_completed
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(r'D:\Documents\Projects\Repositories\MoonCar-mapping').resolve()
AUDIT = ROOT / 'reports' / 'storage-cleanup-20261020'
EXTENSIONS = {'.csv', '.jsonl', '.json', '.log', '.txt', '.patch', '.dump', '.obj', '.ply', '.pgm'}
kernel = ctypes.WinDLL('kernel32', use_last_error=True)
size_on_disk = kernel.GetCompressedFileSizeW
size_on_disk.argtypes = [wintypes.LPCWSTR, ctypes.POINTER(wintypes.DWORD)]
size_on_disk.restype = wintypes.DWORD


def allocated(path):
    high = wintypes.DWORD()
    low = size_on_disk(str(path), ctypes.byref(high))
    if low == 0xFFFFFFFF and ctypes.get_last_error():
        raise ctypes.WinError(ctypes.get_last_error())
    return (high.value << 32) | low


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def compress(path):
    before = allocated(path)
    result = subprocess.run(['compact.exe', '/C', '/I', '/Q', str(path)],
                            capture_output=True)
    after = allocated(path)
    return {'path': str(path.relative_to(ROOT)), 'before_allocated': before,
            'after_allocated': after, 'exit_code': result.returncode,
            **({'diagnostic': (result.stdout + result.stderr).decode('utf-8', 'replace')[:2000]}
               if result.returncode else {})}


def main():
    selected = []
    seen = set()
    skipped = []
    for scope in [ROOT / 'reports' / 'evidence', ROOT / 'tmp']:
        assert scope.resolve().is_relative_to(ROOT)
        for dp, dirs, names in os.walk(scope, followlinks=False):
            dirs[:] = [n for n in dirs if n not in {'terrain-venv', '__pycache__', 'node_modules'}
                       and not Path(dp, n).is_symlink() and not os.path.isjunction(Path(dp, n))]
            for name in names:
                p = Path(dp, name)
                if p.suffix.lower() not in EXTENSIONS or p.is_symlink():
                    continue
                try:
                    st = p.stat()
                except OSError as error:
                    skipped.append({'path': str(p), 'reason': str(error)})
                    continue
                identity = (st.st_dev, st.st_ino)
                if st.st_size < 1024**2 or identity in seen:
                    continue
                assert p.resolve().is_relative_to(scope)
                seen.add(identity)
                selected.append(p)
    selected.sort(key=lambda p: p.stat().st_size, reverse=True)
    # Include large CSV, JSONL, and recent localization observations in content QA.
    samples = selected[:4]
    for extension in ['.csv', '.jsonl', '.json']:
        samples += next(([p] for p in selected if p.suffix == extension and p not in samples), [])
    sample_hashes = {str(p): sha(p) for p in samples}
    print(json.dumps({'phase': 'compressing', 'unique_files': len(selected),
                      'logical_bytes': sum(p.stat().st_size for p in selected), 'workers': 6}), flush=True)
    results = []
    last = time.monotonic()
    with ThreadPoolExecutor(max_workers=6) as pool:
        futures = [pool.submit(compress, p) for p in selected]
        for future in as_completed(futures):
            results.append(future.result())
            if time.monotonic() - last > 25:
                print(json.dumps({'phase': 'compression_progress', 'completed': len(results),
                                  'total': len(selected),
                                  'allocated_bytes_saved_so_far': sum(r['before_allocated'] - r['after_allocated'] for r in results)}), flush=True)
                last = time.monotonic()
    verified = {str(p): sha(p) == sample_hashes[str(p)] for p in samples}
    output = {'method': 'transparent native NTFS compression; original paths and decoded bytes retained',
              'unique_files': len(results), 'workers': 6,
              'before_allocated': sum(r['before_allocated'] for r in results),
              'after_allocated': sum(r['after_allocated'] for r in results),
              'allocated_bytes_saved': sum(r['before_allocated'] - r['after_allocated'] for r in results),
              'errors': [r for r in results if r['exit_code']], 'skipped_preserved': skipped,
              'sample_content_sha256_verified': verified, 'files': results}
    (AUDIT / 'compression.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in output.items() if k != 'files'}), flush=True)
    assert all(verified.values()), 'Compressed record content hash mismatch'


if __name__ == '__main__':
    main()
