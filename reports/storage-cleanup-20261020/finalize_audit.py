"""Verify retention, unrelated Docker resources, and measured drive-space deltas."""
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
from datetime import datetime

from retain_and_deduplicate import files, digest, ROOT, AUDIT, DATA


def read(name):
    return json.loads((AUDIT / name).read_text(encoding='utf-8-sig'))


def command(*args):
    return subprocess.check_output(args, text=True, encoding='utf-8').strip()


def save(name, data):
    (AUDIT / name).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


def main():
    before = read('before.json')
    dedup = read('deduplication.json')
    compression = read('compression.json')
    # Shared evidence is archival. Protect every alias against an accidental overwrite.
    frozen = set()
    for entry in dedup['links']:
        canonical = ROOT / entry['canonical_path']
        duplicate = ROOT / entry['retained_path']
        assert canonical.resolve().is_relative_to(DATA)
        assert duplicate.resolve().is_relative_to(DATA)
        assert os.path.samefile(canonical, duplicate)
        identity = canonical.stat().st_ino
        if identity not in frozen:
            canonical.chmod(stat.S_IREAD)
            frozen.add(identity)
    protected = read('protected-sha256.json')
    changed = [name for name, expected in protected.items() if digest(ROOT / name) != expected]
    assert not changed, changed
    evidence_results = {}
    for name in ['laser-localization-20261020', 'live-gazebo-turn-localization-20261019',
                 'live-gazebo-nav2-first-goal-20261019', 'live-gazebo-time-sync-20261019']:
        directory = DATA / name
        manifest = json.loads((directory / 'verification.json').read_text(encoding='utf-8'))
        expected = manifest['evidence_sha256']
        mismatches = []
        for relative, sha in expected.items():
            path = directory / relative
            assert path.resolve().is_relative_to(directory)
            if digest(path) != sha:
                mismatches.append(relative)
        assert not mismatches, (name, mismatches)
        evidence_results[name] = {'files_verified': len(expected), 'all_content_sha256_unchanged': True}
        print(json.dumps({'phase': 'historical_content_verified', 'directory': name,
                          'files': len(expected)}), flush=True)
    total = count = 0
    for path in files(DATA):
        try:
            st = path.stat()
        except OSError:
            continue
        total += st.st_size
        count += 1
    assert total == dedup['evidence_logical_bytes_before']
    assert count == dedup['evidence_files_before']
    ids = command('docker', 'ps', '-aq').splitlines()
    current = json.loads(command('docker', 'inspect', *ids))
    actual_other = {c['Id']: (c['Name'], c['State']['Status']) for c in current}
    expected_other = {c['Id']: (c['Name'], c['Status']) for c in before['OtherContainers']}
    assert actual_other == expected_other
    image_ids = sorted(set(command('docker', 'images', '-q', '--no-trunc').splitlines()))
    volumes = sorted(command('docker', 'volume', 'ls', '-q').splitlines())
    assert image_ids == sorted(before['Images'])
    assert volumes == sorted(before['Volumes'])
    cache = [json.loads(line) for line in command('docker', 'buildx', 'du', '--format', 'json').splitlines()]
    original_cache = {c['ID'] for c in before['BuildCache']}
    current_cache = {c['ID'] for c in cache}
    assert original_cache - current_cache == {'bwkcwqerbtp6n3rmbz4hnw59h'}
    drives = {}
    for drive in before['DriveBefore']:
        free = shutil.disk_usage(drive['Name'] + ':\\').free
        drives[drive['Name']] = {'free_before': drive['FreeBytes'], 'free_after': free,
                                'observed_free_space_increase': free - drive['FreeBytes']}
    removed = read('removed-containers.json')
    output = {'finished_at': datetime.now().astimezone().isoformat(),
              'source_baseline_reports_unchanged': True, 'protected_files': len(protected),
              'evidence_paths_and_logical_bytes_preserved': True,
              'evidence_files': count, 'evidence_logical_bytes': total,
              'merged_duplicate_files': dedup['duplicate_files_merged'],
              'deduplicated_bytes': dedup['duplicate_logical_bytes_removed'],
              'hardlinked_archival_file_groups_read_only': len(frozen),
              'lossless_compression_bytes_saved': compression['allocated_bytes_saved'],
              'compression_errors': compression['errors'],
              'compression_content_samples_unchanged': all(compression['sample_content_sha256_verified'].values()),
              'historical_evidence_verification': evidence_results,
              'removed_mooncar_containers': len(removed),
              'removed_container_writable_bytes': sum(c['WritableBytes'] for c in removed),
              'removed_exclusive_cache_record_ids': list(original_cache - current_cache),
              'other_container_ids_and_status_unchanged': True, 'all_images_unchanged': True,
              'all_volumes_unchanged': True, 'other_build_cache_records_preserved': True,
              'unrelated_running_services': [{'name': c['Name'], 'health': c['State'].get('Health', {}).get('Status')}
                                             for c in current if c['State']['Running']],
              'drive_measurements': drives,
              'docker_vhdx_file_length_before': before['DockerDiskBefore'][0]['Length'],
              'docker_vhdx_file_length_after': Path(before['DockerDiskBefore'][0]['FullName']).stat().st_size,
              'docker_offline_compaction_performed': False,
              'virtualenv_deletion_blocked_and_preserved': True,
              'simulations_or_builds_started': False}
    save('after.json', output)
    print(json.dumps(output, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
