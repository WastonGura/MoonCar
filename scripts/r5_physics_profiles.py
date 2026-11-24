"""Research integration accuracy is independent of the suspension model."""
PROFILES={'legacy_1ms':.001,'r5_precision_v1':.00025}

def main_step(profile):
    if profile not in PROFILES:raise ValueError('Unknown physics profile')
    return PROFILES[profile]

def profile_record(profile):
    return dict(name=profile,main_step_s=main_step(profile),production_changed=False,
        physical_parameters_changed=False,physics_engine_modified=False)
