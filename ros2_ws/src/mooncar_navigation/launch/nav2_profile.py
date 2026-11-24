"""Profile-aware Nav2 footprint and inflation settings for legacy vs R5 rovers.

References:
- Macenski et al. (IROS 2020): inflation_radius must exceed circumscribed_radius.
- Macenski et al. (Autonomous Robots 2023): Regulated Pure Pursuit cost_scaling_dist
  should align with the costmap inflation envelope.
"""
import copy
import math

NAV2_PROFILES = {
    'legacy': {
        # Legacy mooncar.urdf.xacro outer envelope: 0.84 x 0.70 m (+0.01 m margin -> 0.86 x 0.72 m).
        'half_length_m': 0.43,
        'half_width_m': 0.36,
        'footprint': '[[-0.43, -0.36], [-0.43, 0.36], [0.43, 0.36], [0.43, -0.36]]',
        'inflation_radius_m': 0.65,
        'cost_scaling_factor': 3.0,
        'cost_scaling_dist_m': None,
        'chassis_clearance_m': 0.65,
        'boundary_margin_m': 0.10,
    },
    'r5_support_v1': {
        # mooncar_r5.urdf.xacro outer collision envelope: 0.466 x 0.3811 m (visual wheel caps 0.4011 m).
        # Padded footprint matches mooncar_gazebo/config/laser_filter_r5.yaml: 0.50 x 0.44 m.
        # Inscribed radius = 0.22 m, circumscribed radius = sqrt(0.25^2 + 0.22^2) = 0.3330 m.
        # Inflation radius = 0.45 m (> 0.3330 m + 0.117 m clearance buffer).
        'half_length_m': 0.25,
        'half_width_m': 0.22,
        'footprint': '[[-0.25, -0.22], [-0.25, 0.22], [0.25, 0.22], [0.25, -0.22]]',
        'inflation_radius_m': 0.45,
        'cost_scaling_factor': 4.0,
        'cost_scaling_dist_m': 0.45,
        'chassis_clearance_m': 0.45,
        'boundary_margin_m': 0.10,
    },
}


def profile_geometry(simulation_profile='legacy'):
    if simulation_profile not in NAV2_PROFILES:
        raise ValueError(f'Unknown simulation_profile: {simulation_profile!r}')
    return dict(NAV2_PROFILES[simulation_profile])


def circumscribed_radius(simulation_profile='legacy'):
    geom = profile_geometry(simulation_profile)
    return math.hypot(geom['half_length_m'], geom['half_width_m'])


def apply_nav2_profile(params, simulation_profile='legacy', *, inplace=False):
    """Apply simulation-profile footprint/inflation parameters to a loaded Nav2 dict."""
    geom = profile_geometry(simulation_profile)
    result = params if inplace else copy.deepcopy(params)
    if simulation_profile == 'legacy':
        return result
    for name in ('local_costmap', 'global_costmap'):
        costmap = result[name][name]['ros__parameters']
        costmap['footprint'] = geom['footprint']
        inflation = costmap['inflation_layer']
        inflation['inflation_radius'] = geom['inflation_radius_m']
        inflation['cost_scaling_factor'] = geom['cost_scaling_factor']
    controller = result.get('controller_server', {}).get('ros__parameters', {})
    if geom['cost_scaling_dist_m'] is not None:
        for plugin_name in ('FollowPath', 'RevisitFollowPath', 'SlopeFollowPath'):
            plugin = controller.get(plugin_name)
            if isinstance(plugin, dict):
                plugin['cost_scaling_dist'] = geom['cost_scaling_dist_m']
    return result
