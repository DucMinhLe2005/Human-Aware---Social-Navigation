#!/usr/bin/env python3
"""Export golden data from numpy/scipy themselves for the C++ tests.

The C++ implementation must return exactly what Python returns, and these three
functions are where silent deviations hide (no error, no warning, just a person
centre shifted by centimetres or a different assignment):
  1. np.median / np.percentile  -- lidar cluster centres
  2. linear_sum_assignment      -- track <-> measurement association
  3. distance_transform_edt     -- static obstacle filter

Every float is written with float.hex() (C99 %a) and read back with std::strtod,
so the comparison is bit-exact. The format is deliberately simple: each record
starts with a "--" line followed by "key value value ..." lines, readable with
`std::istringstream >>`.

Run: python3 tools/dump_golden.py   (overwrites test/golden/*.txt)
"""

import os
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.ndimage import distance_transform_edt

GOLDEN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "test", "golden")


def hx(value):
    return float(value).hex()


def hx_list(values):
    return [hx(v) for v in np.asarray(values, dtype=np.float64).ravel()]


def write(name, rows):
    """Flat format: each record starts with "--", followed by "key value..." lines."""
    path = os.path.join(GOLDEN_DIR, name)
    with open(path, "w") as handle:
        for row in rows:
            handle.write("--\n")
            for key, value in row.items():
                if isinstance(value, (list, tuple)):
                    handle.write(f"{key} " + " ".join(str(v) for v in value) + "\n")
                else:
                    handle.write(f"{key} {value}\n")
    print(f"{name}: {len(rows)} ca")


def dump_median_percentile(rng):
    rows = []
    # Lengths 1..12: even (median = MEAN of the two middle values) and odd.
    for n in range(1, 13):
        for _ in range(12):
            values = rng.uniform(0.05, 9.0, size=n)
            rows.append({
                "values": hx_list(values),
                "median": hx(np.median(values)),
                "p20": hx(np.percentile(values, 20.0)),
            })
    # Special cases: all values equal, and values very close together (interpolation).
    for n in (2, 3, 4, 5):
        same = np.full(n, 1.234567)
        rows.append({
            "values": hx_list(same),
            "median": hx(np.median(same)),
            "p20": hx(np.percentile(same, 20.0)),
        })
    near = np.array([1.0, 1.0 + 2 ** -50, 1.0 + 2 ** -49, 1.0 + 2 ** -48])
    rows.append({
        "values": hx_list(near),
        "median": hx(np.median(near)),
        "p20": hx(np.percentile(near, 20.0)),
    })
    write("median_percentile.txt", rows)


def _sentinel_cost(anchors, candidates, gates):
    """Build the cost matrix exactly like human_tracking.py does.

    All out-of-gate pairs get the SAME sentinel value, so the problem has many equal
    optimal solutions -- exactly where two Hungarian implementations can return
    different but equally correct results.
    """
    max_gate = max(gates) if len(gates) else 0.0
    sentinel = max_gate * 1e4 + 1.0
    cost = np.full((len(anchors), len(candidates)), sentinel)
    for i, (ax, ay) in enumerate(anchors):
        for j, (cx, cy) in enumerate(candidates):
            distance = float(np.hypot(ax - cx, ay - cy))
            if distance <= gates[i]:
                cost[i, j] = distance
    return cost


def dump_lsap(rng):
    rows = []

    def emit(cost):
        cost = np.asarray(cost, dtype=np.float64)
        row_ind, col_ind = linear_sum_assignment(cost)
        rows.append({
            "rows": int(cost.shape[0]),
            "cols": int(cost.shape[1]),
            "cost": hx_list(cost),
            "row_ind": [int(v) for v in row_ind],
            "col_ind": [int(v) for v in col_ind],
            "total": hx(cost[row_ind, col_ind].sum()),
        })

    # 1. Random, all shapes -- including rows > cols and rows < cols.
    for n_rows in range(1, 8):
        for n_cols in range(1, 8):
            for _ in range(4):
                emit(rng.uniform(0.0, 5.0, size=(n_rows, n_cols)))

    # 2. Shapes that really occur in the tracker: tracks and candidates along a corridor.
    for n_rows in range(1, 7):
        for n_cols in range(1, 7):
            for _ in range(6):
                anchors = rng.uniform(-4.0, 4.0, size=(n_rows, 2))
                candidates = rng.uniform(-4.0, 4.0, size=(n_cols, 2))
                gates = rng.uniform(0.25, 0.9, size=n_rows)
                emit(_sentinel_cost(anchors, candidates, gates))

    # 3. Deliberately degenerate: many pairs with EXACTLY equal costs.
    for n in (2, 3, 4, 5):
        emit(np.ones((n, n)))
        block = np.ones((n, n)) * 7.0
        block[0, :] = 1.0
        emit(block)
        symmetric = np.array([[abs(i - j) * 1.0 for j in range(n)] for i in range(n)])
        emit(symmetric)

    write("lsap_calls.txt", rows)


def dump_edt(rng):
    rows = []

    def emit(obstacles):
        obstacles = np.asarray(obstacles, dtype=bool)
        # Distance from each cell to the nearest OBSTACLE, via the EDT of the complement
        # (scipy measures the distance to the nearest zero).
        distance = distance_transform_edt(~obstacles)
        rows.append({
            "width": int(obstacles.shape[1]),
            "height": int(obstacles.shape[0]),
            "obstacles": [int(v) for v in obstacles.ravel()],
            "distance": hx_list(distance),
        })

    # A few basic shapes, easy to read when a test fails.
    single = np.zeros((7, 9), dtype=bool)
    single[3, 4] = True
    emit(single)

    wall = np.zeros((10, 12), dtype=bool)
    wall[:, 0] = True
    emit(wall)

    corners = np.zeros((8, 8), dtype=bool)
    corners[0, 0] = corners[0, 7] = corners[7, 0] = corners[7, 7] = True
    emit(corners)

    # Completely EMPTY rows/columns -- exposes infinity handling in the 1-D pass.
    sparse_rows = np.zeros((9, 9), dtype=bool)
    sparse_rows[4, :] = True
    emit(sparse_rows)

    # Random maps of various densities, odd sizes to expose index errors.
    for shape in [(13, 17), (31, 29), (64, 48)]:
        for density in (0.02, 0.1, 0.35):
            emit(rng.uniform(size=shape) < density)

    write("edt_maps.txt", rows)




def _load_python_tracker():
    """Load human_tracking.py from the Python package (it does not import rclpy)."""
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    module_path = os.path.normpath(os.path.join(
        here, "..", "..", "social_nav_tracking", "social_nav_tracking", "human_tracking.py"))
    import sys
    spec = importlib.util.spec_from_file_location("ht_reference", module_path)
    module = importlib.util.module_from_spec(spec)
    # @dataclass looks up sys.modules[cls.__module__] while resolving annotations,
    # so the module must be registered BEFORE executing it.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _synthetic_scan(rng, sensor, n_people, add_wall, noise):
    """Build a synthetic scan: a few people, possibly a wall, and stray beams.

    Not meant to be realistic but to exercise every branch: small clusters accepted,
    wide clusters rejected as walls, single beams rejected, and a one-beam gap inside
    a person (scan_gap <= 2 tolerance).
    """
    points = []
    indices = []
    next_index = int(rng.integers(0, 40))

    for _ in range(n_people):
        angle = rng.uniform(-np.pi, np.pi)
        distance = rng.uniform(1.0, 5.0)
        cx = sensor[0] + distance * np.cos(angle)
        cy = sensor[1] + distance * np.sin(angle)
        n_beams = int(rng.integers(3, 9))
        for k in range(n_beams):
            # Occasionally drop one beam to test the single-bad-beam tolerance.
            if n_beams > 4 and k == n_beams // 2 and rng.uniform() < 0.5:
                next_index += 1
                continue
            spread = (k - n_beams / 2.0) * 0.045
            px = cx + spread * np.cos(angle + np.pi / 2.0) + noise * rng.normal()
            py = cy + spread * np.sin(angle + np.pi / 2.0) + noise * rng.normal()
            points.append((px, py))
            indices.append(next_index)
            next_index += 1
        next_index += int(rng.integers(4, 12))

    if add_wall:
        wall_y = sensor[1] + rng.uniform(2.0, 4.0)
        for k in range(25):
            points.append((sensor[0] - 3.0 + k * 0.12, wall_y + noise * rng.normal()))
            indices.append(next_index)
            next_index += 1
        next_index += 6

    for _ in range(int(rng.integers(0, 4))):
        points.append((sensor[0] + rng.uniform(-6.0, 6.0), sensor[1] + rng.uniform(-6.0, 6.0)))
        indices.append(next_index)
        next_index += int(rng.integers(5, 20))

    return np.asarray(points, dtype=np.float64), np.asarray(indices, dtype=np.int64)


def dump_clustering(rng, ht):
    rows = []
    for case in range(140):
        sensor = (rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0))
        n_people = int(rng.integers(0, 4))
        points, indices = _synthetic_scan(
            rng, sensor, n_people, add_wall=bool(case % 3 == 0), noise=0.004)
        if points.size == 0:
            continue
        params = {
            "min_points": int(rng.integers(2, 5)),
            "max_point_gap": float(rng.uniform(0.08, 0.3)),
            "max_diameter": float(rng.uniform(0.5, 1.0)),
            "person_radius": float(rng.uniform(0.2, 0.35)),
        }
        centers = ht.cluster_lidar_person_centers(
            points, indices, sensor,
            params["min_points"], params["max_point_gap"],
            params["max_diameter"], params["person_radius"])
        flat_centers = []
        for cx, cy in centers:
            flat_centers.extend([hx(cx), hx(cy)])
        rows.append({
            "n_points": int(points.shape[0]),
            "points": hx_list(points),
            "indices": [int(v) for v in indices],
            "sensor": hx_list(np.asarray(sensor)),
            "min_points": params["min_points"],
            "max_point_gap": hx(params["max_point_gap"]),
            "max_diameter": hx(params["max_diameter"]),
            "person_radius": hx(params["person_radius"]),
            "n_centers": len(centers),
            "centers": flat_centers if flat_centers else ["skip"],
        })
    write("cluster_calls.txt", rows)


def dump_static_filter(rng, ht):
    rows = []
    for _ in range(40):
        height = int(rng.integers(8, 20))
        width = int(rng.integers(8, 20))
        resolution = float(rng.choice([0.05, 0.1]))
        grid = np.zeros((height, width), dtype=np.int16)
        grid[rng.uniform(size=grid.shape) < 0.12] = 100
        # A few unknown (-1) cells to exercise the grid >= 0 condition.
        unknown = rng.uniform(size=grid.shape) < 0.08
        grid[unknown] = -1
        obstacles = grid >= 65
        if obstacles.any():
            clearance = distance_transform_edt(~obstacles) * resolution
        else:
            clearance = np.full(grid.shape, np.inf)
        origin_x = float(rng.uniform(-2.0, 0.0))
        origin_y = float(rng.uniform(-2.0, 0.0))
        origin_yaw = float(rng.choice([0.0, 0.3, -0.7]))
        rejection_radius = float(rng.choice([0.0, 0.15, 0.4]))

        n_centers = int(rng.integers(1, 8))
        centers = []
        for _ in range(n_centers):
            centers.append((
                origin_x + float(rng.uniform(-0.5, width * resolution + 0.5)),
                origin_y + float(rng.uniform(-0.5, height * resolution + 0.5)),
            ))
        kept = ht.filter_lidar_person_centers_by_static_map(
            centers, grid, clearance, resolution,
            origin_x, origin_y, origin_yaw, rejection_radius)
        flat_kept = []
        for cx, cy in kept:
            flat_kept.extend([hx(cx), hx(cy)])
        rows.append({
            "width": width,
            "height": height,
            "resolution": hx(resolution),
            "origin_x": hx(origin_x),
            "origin_y": hx(origin_y),
            "origin_yaw": hx(origin_yaw),
            "rejection_radius": hx(rejection_radius),
            "grid": [int(v) for v in grid.ravel()],
            "clearance": hx_list(clearance),
            "n_centers": n_centers,
            "centers": hx_list(np.asarray(centers)),
            "n_kept": len(kept),
            "kept": flat_kept if flat_kept else ["skip"],
        })
    write("static_filter.txt", rows)


def dump_exclusion_zones(rng, ht):
    rows = []
    for _ in range(40):
        n_zones = int(rng.integers(0, 4))
        zones = [(float(rng.uniform(-3, 3)), float(rng.uniform(-3, 3)),
                  float(rng.uniform(0.0, 1.2))) for _ in range(n_zones)]
        n_centers = int(rng.integers(1, 8))
        centers = [(float(rng.uniform(-4, 4)), float(rng.uniform(-4, 4)))
                   for _ in range(n_centers)]
        kept = ht.filter_lidar_person_centers_by_exclusion_zones(centers, zones)
        flat_zones = []
        for zx, zy, zr in zones:
            flat_zones.extend([hx(zx), hx(zy), hx(zr)])
        flat_kept = []
        for cx, cy in kept:
            flat_kept.extend([hx(cx), hx(cy)])
        rows.append({
            "n_zones": n_zones,
            "zones": flat_zones if flat_zones else ["skip"],
            "n_centers": n_centers,
            "centers": hx_list(np.asarray(centers)),
            "n_kept": len(kept),
            "kept": flat_kept if flat_kept else ["skip"],
        })
    write("exclusion_zones.txt", rows)


def main():
    os.makedirs(GOLDEN_DIR, exist_ok=True)
    rng = np.random.default_rng(20260913)
    ht = _load_python_tracker()
    dump_median_percentile(rng)
    dump_lsap(rng)
    dump_edt(rng)
    dump_clustering(rng, ht)
    dump_static_filter(rng, ht)
    dump_exclusion_zones(rng, ht)
    print("numpy", np.__version__)


if __name__ == "__main__":
    main()
