#!/usr/bin/env python3
"""Derive the Nav2 + DWB baseline configuration from the PS-VSP navigation.yaml.

    python3 make_dwb_yaml.py <navigation.yaml> <output.yaml>
                             [--keep-social-layer] [--keep-controller] [--debug]
                             [--static-zone] [--mppi]

Exactly two things change; everything else (AMCL, NavFn A*, goal checker,
inflation, velocity smoother, collision monitor, behaviours) is copied as is:

  1. controller_server.FollowPath: HumanAwareController1 -> dwb_core::DWBLocalPlanner
     with the Nav2 default critics and the same speed and acceleration limits.
  2. The social layer (aghpm_social_layer) is removed from the local and the
     global costmap, so people are seen only as lidar obstacles.

The baseline is therefore a human-unaware Nav2 stack on the same robot.

With --keep-social-layer only change 1 is applied: DWB plans on the same
social costmap as PS-VSP. This ablation separates the contribution of the
local planner from the contribution of the social costmap layer.

With --keep-controller only change 2 is applied: PS-VSP runs without the
social costmap layer (it still receives the tracked humans directly).

With --static-zone the controller and the layer are both kept, and only the
layer's parameters change: the zone becomes a symmetric, static Gaussian
(front = back = side width, no growth with speed or tracker covariance, no
predicted corridor). Radius and cost ceiling are unchanged. This isolates the
asymmetric, adaptive and predictive parts of the Predictive Social Zone.

With --mppi the replacement controller is Nav2's MPPI controller instead of
DWB, with the parameters of nav2_bringup/params/nav2_params.yaml (branch
jazzy) except: the velocity and acceleration limits are those of PS-VSP and
the velocity smoother, and visualisation is off. --mppi combines with
--keep-social-layer (MPPI on the PSZ costmap) or alone (no PSZ).

--debug (DWB only, diagnostic runs, never for reported batches) makes DWB
publish every critic's score on /evaluation.
"""
import sys
import yaml

SOCIAL_LAYER = 'aghpm_social_layer'


def dwb_params(old, smoother, goal_checker):
    """DWB with Nav2's documented default critics; limits taken from the
    PS-VSP configuration so both planners work under the same constraints."""
    v_max = float(old['max_linear_vel'])
    w_max = float(old['max_angular_vel'])
    acc_x, _, acc_w = [float(a) for a in smoother['max_accel']]
    dec_x, _, dec_w = [float(a) for a in smoother['max_decel']]
    return {
        'plugin': 'dwb_core::DWBLocalPlanner',
        'debug_trajectory_details': False,
        'min_vel_x': 0.0,               # Nav2 default: DWB does not reverse
        'min_vel_y': 0.0,
        'max_vel_x': v_max,
        'max_vel_y': 0.0,
        'max_vel_theta': w_max,
        'min_speed_xy': 0.0,
        'max_speed_xy': v_max,
        'min_speed_theta': 0.0,
        'acc_lim_x': acc_x,
        'acc_lim_y': 0.0,
        'acc_lim_theta': acc_w,
        'decel_lim_x': dec_x,
        'decel_lim_y': 0.0,
        'decel_lim_theta': dec_w,
        'vx_samples': 20,
        'vy_samples': 5,
        'vtheta_samples': 20,
        'sim_time': 1.7,
        'linear_granularity': 0.05,
        'angular_granularity': 0.025,
        'transform_tolerance': 0.2,
        'xy_goal_tolerance': float(goal_checker['xy_goal_tolerance']),
        'trans_stopped_velocity': 0.25,
        'short_circuit_trajectory_evaluation': True,
        'stateful': True,
        'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
                    'PathAlign', 'PathDist', 'GoalDist'],
        'BaseObstacle.scale': 0.02,
        'PathAlign.scale': 32.0,
        'PathAlign.forward_point_distance': 0.1,
        'GoalAlign.scale': 24.0,
        'GoalAlign.forward_point_distance': 0.1,
        'PathDist.scale': 32.0,
        'GoalDist.scale': 24.0,
        'RotateToGoal.scale': 32.0,
        'RotateToGoal.slowing_factor': 5.0,
        'RotateToGoal.lookahead_time': -1.0,
    }


def mppi_params(old, smoother):
    """Nav2 MPPI, defaults of nav2_bringup/params/nav2_params.yaml (jazzy branch,
    checked 2026-10-13). Only the limits are taken from the PS-VSP configuration,
    so both controllers work under the same constraints; visualisation is off."""
    acc_x, _, acc_w = [float(a) for a in smoother['max_accel']]
    dec_x, _, _ = [float(a) for a in smoother['max_decel']]
    critic = lambda **kw: dict(enabled=True, cost_power=1, **kw)
    return {
        'plugin': 'nav2_mppi_controller::MPPIController',
        'time_steps': 56, 'model_dt': 0.05, 'batch_size': 2000,
        'ax_max': acc_x, 'ax_min': dec_x, 'ay_max': 3.0, 'ay_min': -3.0, 'az_max': acc_w,
        'vx_std': 0.2, 'vy_std': 0.2, 'wz_std': 0.4,
        'vx_max': float(old['max_linear_vel']), 'vx_min': float(old['min_linear_vel']),
        'vy_max': 0.5, 'wz_max': float(old['max_angular_vel']),
        'iteration_count': 1, 'prune_distance': 1.7, 'transform_tolerance': 0.1,
        'temperature': 0.3, 'gamma': 0.015, 'motion_model': 'DiffDrive',
        'visualize': False, 'regenerate_noises': True,
        'TrajectoryVisualizer': {'trajectory_step': 5, 'time_step': 3},
        'AckermannConstraints': {'min_turning_r': 0.2},
        'critics': ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
                    'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic',
                    'PreferForwardCritic'],
        'ConstraintCritic': critic(cost_weight=4.0),
        'GoalCritic': critic(cost_weight=5.0, threshold_to_consider=1.4),
        'GoalAngleCritic': critic(cost_weight=3.0, threshold_to_consider=0.5),
        'PreferForwardCritic': critic(cost_weight=5.0, threshold_to_consider=0.5),
        'CostCritic': critic(cost_weight=3.81, near_collision_cost=253, critical_cost=300.0,
                             consider_footprint=False, collision_cost=1000000.0,
                             near_goal_distance=1.0, trajectory_point_step=2),
        'PathAlignCritic': critic(cost_weight=14.0, max_path_occupancy_ratio=0.05,
                                  trajectory_point_step=4, threshold_to_consider=0.5,
                                  offset_from_furthest=20, use_path_orientations=False),
        'PathFollowCritic': critic(cost_weight=5.0, offset_from_furthest=5,
                                   threshold_to_consider=1.4),
        'PathAngleCritic': critic(cost_weight=2.0, offset_from_furthest=4,
                                  threshold_to_consider=0.5, max_angle_to_furthest=1.0, mode=0),
    }


STATIC_ZONE = {
    'sigma_back': None, 'sigma_side': None,        # filled with sigma_front
    'velocity_social_enabled': False,
    'velocity_sigma_front_gain': 0.0, 'velocity_sigma_side_gain': 0.0,
    'velocity_sigma_age_gain': 0.0, 'velocity_cost_radius_gain': 0.0,
    'moving_sigma_front_extra': 0.0,
    'social_covariance_enabled': False, 'social_covariance_gain': 0.0,
    'velocity_future_enabled': False,
}


def static_zone(params):
    for name in ('local_costmap', 'global_costmap'):
        layer = params[name][name]['ros__parameters'][SOCIAL_LAYER]
        for key, value in STATIC_ZONE.items():
            if key not in layer:
                raise SystemExit(f'{name}.{SOCIAL_LAYER}: parameter {key} not found')
            layer[key] = layer['sigma_front'] if value is None else value
    return params


def convert(params, keep_social=False, keep_controller=False, debug=False, mppi=False):
    ctrl = params['controller_server']['ros__parameters']
    old = ctrl['FollowPath']
    if 'HumanAwareController' not in old['plugin']:
        raise SystemExit(f"unexpected controller plugin: {old['plugin']}")
    if mppi:
        ctrl['FollowPath'] = mppi_params(old, params['velocity_smoother']['ros__parameters'])
    elif not keep_controller:
        ctrl['FollowPath'] = dwb_params(
            old, params['velocity_smoother']['ros__parameters'], ctrl['general_goal_checker'])
        if debug:
            ctrl['FollowPath']['debug_trajectory_details'] = True
            ctrl['FollowPath']['publish_evaluation'] = True
            ctrl['FollowPath']['publish_trajectories'] = True
    for name in (() if keep_social else ('local_costmap', 'global_costmap')):
        cm = params[name][name]['ros__parameters']
        if SOCIAL_LAYER not in cm['plugins']:
            raise SystemExit(f'{name}: {SOCIAL_LAYER} not in plugins')
        cm['plugins'] = [p for p in cm['plugins'] if p != SOCIAL_LAYER]
        cm.pop(SOCIAL_LAYER, None)
    return params


def changed_keys(a, b, prefix=''):
    keys = []
    for k in sorted(set(a) | set(b), key=str):
        if k not in a or k not in b:
            keys.append(prefix + str(k))
        elif isinstance(a[k], dict) and isinstance(b[k], dict):
            keys += changed_keys(a[k], b[k], prefix + str(k) + '.')
        elif a[k] != b[k]:
            keys.append(prefix + str(k))
    return keys


def main():
    flags = ('--keep-social-layer', '--keep-controller', '--debug', '--static-zone', '--mppi')
    if '--static-zone' in sys.argv:
        if len([a for a in sys.argv if a in flags]) != 1:
            raise SystemExit('--static-zone cannot be combined with other flags')
        argv = [a for a in sys.argv if a not in flags]
        if len(argv) != 3:
            raise SystemExit(__doc__)
        src = yaml.safe_load(open(argv[1]))
        out = static_zone(yaml.safe_load(open(argv[1])))
        prefix = tuple(f'{n}.{n}.ros__parameters.{SOCIAL_LAYER}.' for n in ('local_costmap', 'global_costmap'))
        stray = [k for k in changed_keys(src, out) if not k.startswith(prefix)]
        if stray:
            raise SystemExit(f'unexpected differences: {stray}')
        with open(argv[2], 'w') as f:
            f.write('# GENERATED by analysis/baseline/make_dwb_yaml.py -- PS-VSP, static symmetric zone.\n')
            yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False)
        print(f'wrote {argv[2]}: PS-VSP controller, social layer kept as a static symmetric zone; '
              f'changed {sorted(set(k.rsplit(".", 1)[1] for k in changed_keys(src, out)))}')
        return
    keep = '--keep-social-layer' in sys.argv
    keep_ctrl = '--keep-controller' in sys.argv
    debug = '--debug' in sys.argv
    mppi = '--mppi' in sys.argv
    if mppi and (keep_ctrl or debug):
        raise SystemExit('--mppi combines only with --keep-social-layer')
    if keep and keep_ctrl:
        raise SystemExit('--keep-social-layer with --keep-controller is the unmodified config')
    if debug and keep_ctrl:
        raise SystemExit('--debug applies to DWB only')
    argv = [a for a in sys.argv if a not in flags]
    if len(argv) != 3:
        raise SystemExit(__doc__)
    sys.argv = argv
    src = yaml.safe_load(open(sys.argv[1]))
    out = convert(yaml.safe_load(open(sys.argv[1])), keep, keep_ctrl, debug, mppi)
    allowed = ('controller_server.ros__parameters.FollowPath',
               'local_costmap.local_costmap.ros__parameters.plugins',
               'local_costmap.local_costmap.ros__parameters.' + SOCIAL_LAYER,
               'global_costmap.global_costmap.ros__parameters.plugins',
               'global_costmap.global_costmap.ros__parameters.' + SOCIAL_LAYER)
    stray = [k for k in changed_keys(src, out) if not k.startswith(allowed)]
    if stray:
        raise SystemExit(f'unexpected differences: {stray}')
    with open(sys.argv[2], 'w') as f:
        f.write('# GENERATED by analysis/baseline/make_dwb_yaml.py -- '
                + ('PS-VSP' if keep_ctrl else 'MPPI' if mppi else 'DWB') + ', social layer '
                + ('kept' if keep else 'removed') + (', DEBUG' if debug else '') + '.\n')
        yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False)
    print(f'wrote {sys.argv[2]}: ' + ('PS-VSP' if keep_ctrl else 'MPPI' if mppi else 'DWB') + ' controller, social layer '
          + ('kept in both costmaps' if keep else 'removed from both costmaps')
          + (' -- DEBUG, not for reported batches' if debug else ''))


if __name__ == '__main__':
    main()
