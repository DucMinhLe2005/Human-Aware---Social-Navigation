#!/usr/bin/env python3
"""Offline analysis of a batch produced by run_batch.sh -- no change to the
simulation workspace is needed.

Run the batch with KEEP_BAG=1 so every run keeps its rosbag:
    KEEP_BAG=1 ./run_batch.sh 100

Then (in a terminal where ROS 2 Jazzy and the workspace are sourced):
    python3 analyze_batch.py <batch_dir> --method "PS-VSP" --scenario dymap

Every run directory is counted, including the ones run_batch.sh printed as
"NOT COUNTED" (they are kept on disk with their goal.txt). Each run gets one
outcome:
    SUCCESS    goal reached, no collision
    COLLISION  at least one contact with a person
    TIMEOUT    send_goal.py gave up
    ABORTED    Nav2 ended the goal without success
    INVALID    infrastructure failure (goal never accepted / robot moved < 1 m);
               excluded from N and reported separately

Metrics (per run, over the GOAL WINDOW = goal start to goal end in simulation
time, as logged in metrics.json; runs without a logged goal fall back to first
non-zero velocity command -> end of the bag, and --window command forces that
older definition; the window used is in the 'window' column of runs.csv):
    SZVR  time with clearance to any person < zone / window length
    MHC   minimum robot-person clearance (body shapes, not centres)
    PL    robot path length from ground truth
    TT    window length, reported for successful runs
    smoothness  mean |acc| and mean |jerk| of commanded v and w, full stops,
                sign changes of w

Outputs in <batch_dir>/analysis/: runs.csv (one row per run), summary.json,
summary.txt. Runs without a bag fall back to metrics.json (collisions, MHC and
distance only).

Self-test without ROS:  python3 analyze_batch.py --self-test
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys

# ----------------------------------------------------------------- defaults
# Shapes as in social_nav_metrics.py (dymap.sdf). Override with --people.
DEFAULT_PEOPLE = [
    'actor_collision_proxy=circle:0.28',
    'Nurse=box:-0.111,0.511,-0.246,0.200',
    'Nurse_clone=box:-0.111,0.511,-0.246,0.200',
]
INFRA_PREFIXES = ('NO navigate', 'GOAL_REJECTED', 'GOAL_RESPONSE_LOST')


# ------------------------------------------------------------- 2D geometry
# Same geometry as social_nav_metrics.py so that numbers are comparable.
def to_world(x, y, yaw, px, py):
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * px - s * py, y + s * px + c * py)


def box_polygon(x, y, yaw, box):
    xmin, xmax, ymin, ymax = box
    return [to_world(x, y, yaw, px, py)
            for px, py in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))]


def point_segment_distance(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + u * dx), p[1] - (ay + u * dy))


def edges(poly):
    return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]


def inside_convex(p, poly):
    sign = 0
    for a, b in edges(poly):
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(cross) < 1e-12:
            continue
        s = 1 if cross > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return True


def segments_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return orient(a, b, c) * orient(a, b, d) < 0 and orient(c, d, a) * orient(c, d, b) < 0


def clearance_polygon_circle(poly, center, radius):
    """Clearance (m). Negative = overlapping."""
    d = min(point_segment_distance(center, a, b) for a, b in edges(poly))
    return (-d if inside_convex(center, poly) else d) - radius


def clearance_polygon_polygon(p, q):
    """Clearance (m). 0 = touching or overlapping."""
    if any(inside_convex(v, q) for v in p) or any(inside_convex(v, p) for v in q):
        return 0.0
    if any(segments_cross(a, b, c, d) for a, b in edges(p) for c, d in edges(q)):
        return 0.0
    return min(min(point_segment_distance(v, a, b) for v in p for a, b in edges(q)),
               min(point_segment_distance(v, a, b) for v in q for a, b in edges(p)))


def parse_shape(spec):
    """'name=circle:r' or 'name=box:xmin,xmax,ymin,ymax' (model frame)."""
    name, shape = spec.split('=', 1)
    kind, values = shape.split(':', 1)
    numbers = [float(v) for v in values.split(',')]
    return name, ((kind, numbers[0]) if kind == 'circle' else (kind, tuple(numbers)))


# ------------------------------------------------------------- core metrics
def analyse_run(gt, cmd, people, robot_name='my_amr', robot_box=(-0.135, 0.135, -0.12, 0.12),
                zone=0.5, collision_release=0.05, max_speed=2.0,
                stop_v=0.01, stop_w=0.02, w_deadband=0.05, t_start=None, t_end=None):
    """gt:  list of (t, {model: (x, y, yaw)})   ground truth, sim time
    cmd: list of (t, v, w)                      velocity commands, sim time
    people: {name: shape}
    t_start, t_end: analysis window in sim time (the goal window). When not
        given, the window is first non-zero command -> end of the bag.
    Returns a dict of per-run metrics, or None if there is no usable data."""
    gt = sorted((g for g in gt if robot_name in g[1]), key=lambda g: g[0])
    cmd = sorted(cmd, key=lambda c: c[0])
    if len(gt) < 2:
        return None
    moving_cmds = [c for c in cmd if abs(c[1]) > stop_v or abs(c[2]) > stop_w]
    t0 = moving_cmds[0][0] if moving_cmds else gt[0][0]
    t1 = gt[-1][0]
    if t_start is not None and t_end is not None:
        t0, t1 = max(t_start, gt[0][0]), min(t_end, gt[-1][0])
    if t1 <= t0:
        return None

    state = {n: {'min': math.inf, 'zone_s': 0.0, 'collisions': 0, 'in': False} for n in people}
    path = 0.0
    zone_any = 0.0
    window = 0.0
    prev_t = prev_pose = None
    prev_any = False
    for t, poses in gt:
        if t < t0:
            continue
        if t > t1:
            break
        pose = poses[robot_name]
        if prev_t is not None:
            dt = t - prev_t
            if 0.0 < dt < 0.5:
                step = math.hypot(pose[0] - prev_pose[0], pose[1] - prev_pose[1])
                if step / dt <= max_speed:          # ignore teleports / resets
                    path += step
                window += dt
                if prev_any:
                    zone_any += dt
        robot_poly = box_polygon(*pose, robot_box)
        any_in = False
        for name, shape in people.items():
            if name not in poses:
                continue
            p = poses[name]
            if shape[0] == 'circle':
                c = clearance_polygon_circle(robot_poly, p[:2], shape[1])
            else:
                c = clearance_polygon_polygon(robot_poly, box_polygon(*p, shape[1]))
            s = state[name]
            s['min'] = min(s['min'], c)
            if c < zone:
                any_in = True
                if prev_t is not None and 0.0 < t - prev_t < 0.5:
                    s['zone_s'] += t - prev_t
            if not s['in'] and c <= 0.0:
                s['in'] = True
                s['collisions'] += 1
            elif s['in'] and c > collision_release:
                s['in'] = False
        prev_t, prev_pose, prev_any = t, pose, any_in

    # Command smoothness inside the window.
    sum_av = sum_aw = sum_jv = sum_jw = T = 0.0
    stops = sign_changes = 0
    last = None
    last_av = last_aw = None
    stopped = True
    w_sign = 0
    for t, v, w in cmd:
        if t > t1:
            continue
        if t < t0:
            last = (t, v, w)        # keeps the step from rest into the first command
            continue
        if last is not None:
            dt = t - last[0]
            if 0.005 < dt < 0.5:
                av, aw = (v - last[1]) / dt, (w - last[2]) / dt
                T += dt
                sum_av += abs(av) * dt
                sum_aw += abs(aw) * dt
                if last_av is not None:
                    sum_jv += abs(av - last_av)      # |jerk| * dt
                    sum_jw += abs(aw - last_aw)
                last_av, last_aw = av, aw
            else:
                last_av = last_aw = None
        is_stopped = abs(v) < stop_v and abs(w) < stop_w
        if is_stopped and not stopped:
            stops += 1
        stopped = is_stopped
        sign = 0 if abs(w) < w_deadband else (1 if w > 0 else -1)
        if sign:
            if w_sign and sign != w_sign:
                sign_changes += 1
            w_sign = sign
        last = (t, v, w)

    mins = [s['min'] for s in state.values() if math.isfinite(s['min'])]
    mean = (lambda x: x / T) if T > 0 else (lambda x: None)
    return {
        'window_s': window,
        'path_length_m': path,
        'szvr': zone_any / window if window > 0 else None,
        'mhc_m': min(mins) if mins else None,
        'collisions': sum(s['collisions'] for s in state.values()),
        'per_person': {n: {'min_clearance_m': (s['min'] if math.isfinite(s['min']) else None),
                           'zone_s': s['zone_s'], 'collisions': s['collisions']}
                       for n, s in state.items()},
        'mean_abs_lin_acc': mean(sum_av), 'mean_abs_ang_acc': mean(sum_aw),
        'mean_abs_lin_jerk': mean(sum_jv), 'mean_abs_ang_jerk': mean(sum_jw),
        'stops': stops, 'omega_sign_changes': sign_changes,
        'cmd_samples': sum(1 for c in cmd if t0 <= c[0] <= t1),
    }


def goal_window(metrics):
    """(start, end) of the navigation goal in sim time, as logged by
    social_nav_metrics in metrics.json; None when it is not available."""
    goals = [g for g in (metrics.get('goals') or [])
             if isinstance(g, dict) and g.get('start') is not None and g.get('duration_s')]
    if not goals:
        return None
    done = [g for g in goals if g.get('result') == 'SUCCEEDED']
    g = (done or goals)[-1]
    return float(g['start']), float(g['start']) + float(g['duration_s'])


def outcome_of(goal, collisions, distance):
    if not goal or goal.startswith(INFRA_PREFIXES) or distance < 1.0:
        return 'INVALID'
    if collisions > 0:
        return 'COLLISION'
    if goal.startswith('GOAL_REACHED'):
        return 'SUCCESS'
    return 'TIMEOUT' if goal.startswith('TIMEOUT') else 'ABORTED'


# ------------------------------------------------------------------ rosbag
def read_bag(bag_dir, gt_topic, cmd_topic):
    """Read ground truth and velocity commands from a rosbag2 directory.

    Needs a sourced ROS 2 environment (rosbag2_py). Command times are the bag
    receive times, which are simulation time because run_batch.sh records with
    --use-sim-time; ground-truth times are the header stamps (also sim time)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    metadata = os.path.join(bag_dir, 'metadata.yaml')
    storage_id = 'mcap'
    if os.path.exists(metadata):
        for line in open(metadata):
            if 'storage_identifier' in line:
                storage_id = line.split(':', 1)[1].strip()
                break
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[gt_topic, cmd_topic]))
    classes = {name: get_message(types[name]) for name in (gt_topic, cmd_topic) if name in types}
    gt, cmd = [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        if topic == gt_topic:
            poses, t = {}, None
            for tf in msg.transforms:
                tr, q = tf.transform.translation, tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                poses[tf.child_frame_id] = (tr.x, tr.y, yaw)
                t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if t is not None:
                gt.append((t, poses))
        else:
            twist = msg.twist if hasattr(msg, 'twist') else msg
            cmd.append((stamp_ns * 1e-9, twist.linear.x, twist.angular.z))
    return gt, cmd


# ------------------------------------------------------------------- batch
def run_index(path):
    tail = os.path.basename(path).rsplit('_', 1)[-1]
    return int(tail) if tail.isdigit() else 0


def load_batch(batch_dir, people, args):
    rows = []
    for run_dir in sorted(glob.glob(os.path.join(batch_dir, 'run_*')), key=run_index):
        if not os.path.isdir(run_dir):
            continue
        goal_path = os.path.join(run_dir, 'goal.txt')
        goal = open(goal_path).read().strip() if os.path.exists(goal_path) else ''
        metrics = {}
        for candidate in [os.path.join(run_dir, 'metrics.json')] + \
                sorted(glob.glob(os.path.join(run_dir, 'run_*.json'))):
            if os.path.exists(candidate):
                metrics = json.load(open(candidate))
                break
        m = None
        source = 'none'
        window_kind = ''
        bag = os.path.join(run_dir, 'bag')
        if os.path.isdir(bag) and not args.no_bag:
            try:
                gt, cmd = read_bag(bag, args.gt_topic, args.cmd_topic)
                box = (-args.robot_length / 2, args.robot_length / 2,
                       -args.robot_width / 2, args.robot_width / 2)
                gw = goal_window(metrics) if args.window == 'goal' else None
                if gw:
                    m = analyse_run(gt, cmd, people, args.robot, box, args.zone,
                                    t_start=gw[0], t_end=gw[1])
                    window_kind = 'goal'
                if m is None:       # no goal logged, or the bag does not cover it
                    m = analyse_run(gt, cmd, people, args.robot, box, args.zone)
                    window_kind = 'command'
                source = 'bag' if m else 'none'
            except Exception as exc:        # keep going; report the run as metrics-only
                print(f'  {os.path.basename(run_dir)}: cannot read bag ({exc})', file=sys.stderr)
        if m is None:
            m = {'window_s': None, 'path_length_m': metrics.get('robot_distance_m'),
                 'szvr': None, 'mhc_m': metrics.get('min_clearance_m'),
                 'collisions': metrics.get('collisions_total', 0),
                 'mean_abs_lin_acc': None, 'mean_abs_ang_acc': None,
                 'mean_abs_lin_jerk': None, 'mean_abs_ang_jerk': None,
                 'stops': None, 'omega_sign_changes': None, 'per_person': {}}
            if metrics:
                source = 'metrics.json'
        node_collisions = metrics.get('collisions_total') if metrics else None
        collisions = max(m['collisions'], node_collisions or 0)
        distance = m['path_length_m'] if m['path_length_m'] is not None else 0.0
        outcome = outcome_of(goal, collisions, distance)
        standing = sum(v['collisions'] for k, v in m.get('per_person', {}).items()
                       if not k.startswith('actor'))
        rows.append({
            'run': os.path.basename(run_dir), 'scenario': args.scenario, 'method': args.method,
            'outcome': outcome, 'goal_txt': goal, 'data_source': source,
            'window': window_kind if source == 'bag' else '',
            'collisions': collisions, 'collisions_metrics_node': node_collisions,
            'standing_collisions': standing,
            'szvr': m['szvr'], 'mhc_m': m['mhc_m'], 'path_length_m': m['path_length_m'],
            'travel_time_s': m['window_s'] if outcome == 'SUCCESS' else None,
            'mean_abs_lin_acc': m['mean_abs_lin_acc'], 'mean_abs_ang_acc': m['mean_abs_ang_acc'],
            'mean_abs_lin_jerk': m['mean_abs_lin_jerk'], 'mean_abs_ang_jerk': m['mean_abs_ang_jerk'],
            'stops': m['stops'], 'omega_sign_changes': m['omega_sign_changes'],
        })
    return rows


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'), float('nan'))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def rate(k, n):
    lo, hi = wilson(k, n)
    return {'count': k, 'rate': k / n if n else float('nan'), 'ci95': [lo, hi]}


def dist(values):
    v = sorted(x for x in values if x is not None)
    if not v:
        return None
    if len(v) == 1:
        return {'n': 1, 'median': v[0], 'q1': v[0], 'q3': v[0], 'p5': v[0]}
    q = statistics.quantiles(v, n=20, method='inclusive')
    return {'n': len(v), 'median': statistics.median(v), 'q1': q[4], 'q3': q[14], 'p5': q[0]}


def summarise(rows):
    runs = [r for r in rows if r['outcome'] != 'INVALID']
    n = len(runs)
    ok = [r for r in runs if r['outcome'] == 'SUCCESS']
    return {
        'N': n, 'invalid_excluded': len(rows) - n,
        'runs_with_bag': sum(r['data_source'] == 'bag' for r in runs),
        'SR': rate(len(ok), n),
        'CR': rate(sum(r['collisions'] > 0 for r in runs), n),
        'timeout': rate(sum(r['outcome'] == 'TIMEOUT' for r in runs), n),
        'aborted': rate(sum(r['outcome'] == 'ABORTED' for r in runs), n),
        'collisions_total': sum(r['collisions'] for r in runs),
        'standing_people_collisions': sum(r['standing_collisions'] for r in runs),
        'SZVR': dist(r['szvr'] for r in runs),
        'MHC_m': dist(r['mhc_m'] for r in runs),
        'PL_m': dist(r['path_length_m'] for r in ok),
        'TT_s': dist(r['travel_time_s'] for r in ok),
        'mean_abs_lin_acc': dist(r['mean_abs_lin_acc'] for r in runs),
        'mean_abs_ang_acc': dist(r['mean_abs_ang_acc'] for r in runs),
        'mean_abs_lin_jerk': dist(r['mean_abs_lin_jerk'] for r in runs),
        'mean_abs_ang_jerk': dist(r['mean_abs_ang_jerk'] for r in runs),
        'stops': dist(r['stops'] for r in runs),
        'omega_sign_changes': dist(r['omega_sign_changes'] for r in runs),
    }


def f_rate(r):
    return f"{100 * r['rate']:.1f}% ({r['count']}) [95% CI {100 * r['ci95'][0]:.1f}-{100 * r['ci95'][1]:.1f}%]"


def f_dist(d, unit='', scale=1.0, digits=2):
    if d is None:
        return 'not available'
    return (f"{scale * d['median']:.{digits}f} [{scale * d['q1']:.{digits}f}, "
            f"{scale * d['q3']:.{digits}f}]{unit} (n={d['n']})")


def report(s, args):
    lines = [
        f"scenario / method      : {args.scenario} / {args.method}",
        f"analysis window        : {s.get('windows', {})}",
        f"N counted runs         : {s['N']}  (+{s['invalid_excluded']} invalid, excluded; "
        f"{s['runs_with_bag']} with a bag)",
        f"SR  success rate       : {f_rate(s['SR'])}",
        f"CR  collision rate     : {f_rate(s['CR'])}",
        f"    timeout            : {f_rate(s['timeout'])}",
        f"    aborted            : {f_rate(s['aborted'])}",
        f"collisions, total      : {s['collisions_total']} "
        f"(with standing people: {s['standing_people_collisions']})",
        f"SZVR  median [Q1, Q3]  : {f_dist(s['SZVR'], ' %', 100.0, 1)}",
        f"MHC   median [Q1, Q3]  : {f_dist(s['MHC_m'], ' m')}",
        f"MHC   5th percentile   : " + (f"{s['MHC_m']['p5']:.2f} m" if s['MHC_m'] else 'not available'),
        f"PL    (success only)   : {f_dist(s['PL_m'], ' m')}",
        f"TT    (success only)   : {f_dist(s['TT_s'], ' s', 1.0, 1)}",
        f"mean |lin acc|         : {f_dist(s['mean_abs_lin_acc'], ' m/s^2', 1.0, 3)}",
        f"mean |ang acc|         : {f_dist(s['mean_abs_ang_acc'], ' rad/s^2', 1.0, 3)}",
        f"mean |lin jerk|        : {f_dist(s['mean_abs_lin_jerk'], ' m/s^3', 1.0, 3)}",
        f"mean |ang jerk|        : {f_dist(s['mean_abs_ang_jerk'], ' rad/s^3', 1.0, 3)}",
        f"full stops per run     : {f_dist(s['stops'], '', 1.0, 0)}",
        f"omega sign changes/run : {f_dist(s['omega_sign_changes'], '', 1.0, 0)}",
    ]
    return '\n'.join(lines)


# ---------------------------------------------------------------- self-test
def self_test():
    people = dict(parse_shape(s) for s in DEFAULT_PEOPLE)
    box = (-0.135, 0.135, -0.12, 0.12)
    # Robot drives +x at 0.4 m/s for 10 s from t=2; a pedestrian (r=0.28) stands
    # 0.6 m to the side of the path at x=2.
    gt, cmd = [], []
    for i in range(0, 1300):
        t = i * 0.01
        x = 0.0 if t < 2 else min(4.0, 0.4 * (t - 2))
        gt.append((t, {'my_amr': (x, 0.0, 0.0), 'actor_collision_proxy': (2.0, 0.6, 0.0)}))
    for i in range(0, 260):
        t = i * 0.05
        v = 0.0 if t < 2 or t >= 12 else 0.4
        cmd.append((t, v, 0.0))
    m = analyse_run(gt, cmd, people, 'my_amr', box)
    assert abs(m['path_length_m'] - 4.0) < 0.02, m['path_length_m']
    expect_mhc = 0.6 - 0.12 - 0.28            # side of the robot box to the circle
    assert abs(m['mhc_m'] - expect_mhc) < 0.01, (m['mhc_m'], expect_mhc)
    assert m['collisions'] == 0
    # In zone while clearance < 0.5: centre within sqrt(0.9^2-0.6^2)... compare numerically.
    inside = sum(0.01 for t, p in gt if t >= 2 and clearance_polygon_circle(
        box_polygon(*p['my_amr'], box), (2.0, 0.6), 0.28) < 0.5)
    assert abs(m['szvr'] * m['window_s'] - inside) < 0.05, (m['szvr'] * m['window_s'], inside)
    assert m['stops'] == 1 and m['omega_sign_changes'] == 0, (m['stops'], m['omega_sign_changes'])
    # One step of 0.4 m/s up and one down over 0.05 s -> mean |acc| = 0.8 / window.
    assert abs(m['mean_abs_lin_acc'] * 11.0 - 0.8) < 0.1, m['mean_abs_lin_acc']
    # Head-on contact.
    gt2 = [(i * 0.01, {'my_amr': (0.4 * i * 0.01, 0.0, 0.0), 'actor_collision_proxy': (2.0, 0.0, 0.0)})
           for i in range(600)]
    m2 = analyse_run(gt2, [(i * 0.05, 0.4, 0.0) for i in range(120)], people, 'my_amr', box)
    assert m2['collisions'] == 1 and m2['mhc_m'] < 0, m2
    assert outcome_of('GOAL_REACHED | 51.0s', 0, 12.0) == 'SUCCESS'
    assert outcome_of('GOAL_REACHED | 51.0s', 1, 12.0) == 'COLLISION'
    assert outcome_of('TIMEOUT after 200s', 0, 6.0) == 'TIMEOUT'
    assert outcome_of('GOAL_ENDED status=6 | 30.0s', 0, 6.0) == 'ABORTED'
    assert outcome_of('NO navigate_to_pose action server', 0, 0.0) == 'INVALID'
    assert outcome_of('TIMEOUT after 200s', 0, 0.3) == 'INVALID'
    lo, hi = wilson(91, 100)
    assert abs(lo - 0.8377) < 1e-3 and abs(hi - 0.9519) < 1e-3
    print('self-test passed')


def self_test_goal_window():
    people = {'h': ('circle', 0.28)}
    gt = [(i * 0.05, {'my_amr': (0.2 * i * 0.05, 0.0, 0.0), 'h': (50.0, 0.0, 0.0)})
          for i in range(401)]                       # 20 s at 0.2 m/s
    cmd = [(i * 0.05, 0.2, 0.0) for i in range(401)]
    m = analyse_run(gt, cmd, people, t_start=5.0, t_end=15.0)
    assert abs(m['window_s'] - 10.0) < 0.06, m['window_s']
    assert abs(m['path_length_m'] - 2.0) < 0.02, m['path_length_m']
    assert goal_window({'goals': [{'start': 3.0, 'duration_s': 4.0, 'result': 'SUCCEEDED'}]}) == (3.0, 7.0)
    assert goal_window({}) is None
    print('goal-window self-test passed')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('batch_dir', nargs='?')
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--method', default='unspecified')
    ap.add_argument('--scenario', default='dymap')
    ap.add_argument('--zone', type=float, default=0.5, help='social zone threshold (m)')
    ap.add_argument('--people', nargs='*', default=DEFAULT_PEOPLE,
                    help="shapes, e.g. actor_collision_proxy=circle:0.28")
    ap.add_argument('--robot', default='my_amr')
    ap.add_argument('--robot-length', type=float, default=0.27)
    ap.add_argument('--robot-width', type=float, default=0.24)
    ap.add_argument('--gt-topic', default='/social_nav/ground_truth')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--window', choices=('goal', 'command'), default='goal',
                    help='goal: goal start -> goal end from metrics.json (default); '
                         'command: first non-zero command -> end of bag')
    ap.add_argument('--no-bag', action='store_true', help='use metrics.json only')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return self_test_goal_window()
    if not args.batch_dir:
        ap.error('batch_dir is required')
    people = dict(parse_shape(s) for s in args.people)
    rows = load_batch(args.batch_dir, people, args)
    if not rows:
        print('no run_* directories found')
        return
    out = os.path.join(args.batch_dir, 'analysis')
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    s = summarise(rows)
    s['windows'] = {k: sum(1 for r in rows if r['window'] == k)
                    for k in sorted({r['window'] for r in rows if r['window']})}
    json.dump({'scenario': args.scenario, 'method': args.method, **s},
              open(os.path.join(out, 'summary.json'), 'w'), indent=2)
    text = report(s, args)
    open(os.path.join(out, 'summary.txt'), 'w').write(text + '\n')
    print(text)
    print(f'\nwritten: {out}/runs.csv, summary.json, summary.txt')


if __name__ == '__main__':
    main()
