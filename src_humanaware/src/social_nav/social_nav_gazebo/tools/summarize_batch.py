#!/usr/bin/env python3
"""Summarise a batch produced by run_batch.sh.

Only valid runs (GOAL_REACHED) are counted. Reports the clean-run rate with a 95%
confidence interval, collisions per run, the clearance to the walking pedestrian
(median and share of runs closer than 0.30 m), collisions with standing people
and the time to goal.

Usage: python3 summarize_batch.py <batch_dir> [--json out.json]
"""
import argparse
import glob
import json
import math
import os
import statistics


def load_runs(batch_dir):
    runs = []
    for metrics_path in sorted(glob.glob(os.path.join(batch_dir, 'run_*', 'metrics.json'))):
        run_dir = os.path.dirname(metrics_path)
        goal_path = os.path.join(run_dir, 'goal.txt')
        if not os.path.exists(goal_path):
            continue
        goal = open(goal_path).read().strip()
        if not goal.startswith('GOAL_REACHED'):
            continue
        with open(metrics_path) as f:
            metrics = json.load(f)
        walking = [p for p in metrics.get('people', []) if p['name'].startswith('actor')]
        standing = [p for p in metrics.get('people', []) if not p['name'].startswith('actor')]
        runs.append({
            'run': os.path.basename(run_dir),
            'collisions': metrics.get('collisions_total', 0),
            'walking_clearance': walking[0]['min_clearance_m'] if walking else None,
            'standing_collisions': sum(p['collisions'] for p in standing),
            'time_to_goal_s': float(goal.split('|')[1].strip().rstrip('s')),
        })
    return runs


def wilson_interval(successes, n, z=1.96):
    if n == 0:
        return (float('nan'), float('nan'))
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (centre - half, centre + half)


def summarise(runs):
    n = len(runs)
    clean = sum(r['collisions'] == 0 for r in runs)
    clearances = [r['walking_clearance'] for r in runs if r['walking_clearance'] is not None]
    low, high = wilson_interval(clean, n)
    return {
        'valid_runs': n,
        'clean_runs': clean,
        'clean_rate': clean / n if n else float('nan'),
        'clean_rate_ci95': [low, high],
        'failed_runs': n - clean,
        'collisions_total': sum(r['collisions'] for r in runs),
        'collisions_per_run': sum(r['collisions'] for r in runs) / n if n else float('nan'),
        'walking_clearance_median_m': statistics.median(clearances) if clearances else None,
        'walking_clearance_below_0_30_rate':
            sum(c < 0.30 for c in clearances) / len(clearances) if clearances else None,
        'standing_people_collisions': sum(r['standing_collisions'] for r in runs),
        'time_to_goal_median_s': statistics.median(r['time_to_goal_s'] for r in runs) if runs else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('batch_dir')
    parser.add_argument('--json')
    args = parser.parse_args()
    s = summarise(load_runs(args.batch_dir))
    if not s['valid_runs']:
        print('no valid runs found')
        return
    print(f"valid runs                    : {s['valid_runs']}")
    print(f"clean runs (no collision)     : {s['clean_runs']} = {100 * s['clean_rate']:.1f}% "
          f"[95% CI {100 * s['clean_rate_ci95'][0]:.1f}-{100 * s['clean_rate_ci95'][1]:.1f}%]")
    print(f"failed runs                   : {s['failed_runs']}")
    print(f"collisions (total / per run)  : {s['collisions_total']} / {s['collisions_per_run']:.3f}")
    print(f"pedestrian clearance (median) : {s['walking_clearance_median_m']:+.3f} m")
    print(f"runs with clearance < 0.30 m  : {100 * s['walking_clearance_below_0_30_rate']:.1f}%")
    print(f"collisions with standing people: {s['standing_people_collisions']}")
    print(f"time to goal (median)         : {s['time_to_goal_median_s']:.1f} s")
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(s, f, indent=2)


if __name__ == '__main__':
    main()
