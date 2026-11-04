"""Frozen research monitoring contract, independent of physics and control."""
def monitoring_profile():
    return dict(name='host_files_v1',transport='host_mount_readonly',container_exec_during_observation=False)

def validate_monitoring(manifest):
    if manifest.get('monitoring_profile')!=monitoring_profile():
        raise ValueError('Research monitoring must use frozen host_files_v1; no exec polling during observation')
    return manifest['monitoring_profile']
