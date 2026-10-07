# Results

## Scenario

Gazebo world `dymap`: the robot starts at (0.5, 0.0) and drives to the goal
(11.18, 2.77) through a doorway. Two people stand still near the doorway and one
pedestrian walks back and forth (0.7 m/s, x = 8.0, y 0.0 <-> 3.0) across the
robot's path, stopping for 1 s to turn around at each end. The pedestrian does
not react to the robot (scripted actor), so it can walk into the robot.

## Protocol

- 100 valid runs; before every run Gazebo, navigation and perception are
  restarted and AMCL is reset from ground truth.
- A run is valid only if the robot reaches the goal. A run with at least one
  collision is a failed run, regardless of the number of collisions.
- Collisions and clearances are computed from ground-truth positions by
  `social_nav_metrics` (clearance between the robot body and the person's
  collision shape; negative = contact).

## Final configuration (100 runs)

| Metric | Value |
|---|---|
| Valid runs | 100 |
| Clean runs (no collision) | **91 (91%, 95% CI 84-95%)** |
| Failed runs | 9 |
| Collisions (total / per run) | 12 / 0.12 |
| Collisions with standing people | **0** |
| Clearance to the pedestrian (median of per-run minimum) | 0.56 m |
| Runs with clearance < 0.30 m | 28% |
| Time to goal (median) | 51.4 s |

Raw numbers: [final_100_runs.json](final_100_runs.json).

All remaining collisions involve the walking pedestrian at the moment it turns
around close to the robot. The tracker's constant-velocity Kalman filter needs
about 1.2 s to follow a reversal, and the camera often cannot see the person at
that angle, so the reversal is detected late.

## Note on variance

Two batches of 100 runs with identical code differed by up to ~9 points in the
clean-run rate (87% vs 78%), more than any effect measured between variants. The
clean-run rate of a single batch should therefore be read together with its
confidence interval, and continuous metrics (clearance distribution) are more
reliable than the binary collision count for comparing variants.

## Evaluated and removed: escape commitment

A decision layer on top of the DWA ("escape commitment": deciding early and
firmly between driving forward and backing off when a pedestrian approaches,
with variants using raw lidar clusters and a social-zone trigger) was
implemented and evaluated over several batches. Against the plain DWA on the
same binary it showed no measurable benefit (median clearance 0.45 vs 0.49 m,
p = 0.65; collisions per run 0.21 vs 0.20), and the social-zone variant
increased decision oscillation and reduced the clearance. It was therefore
removed; the configuration above is the plain human-aware DWA.
