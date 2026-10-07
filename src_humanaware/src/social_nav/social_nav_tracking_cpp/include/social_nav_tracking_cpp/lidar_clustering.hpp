// Groups lidar beams into person candidates and rejects candidates that are
// static map features.
//
// Lidar points lie on the VISIBLE surface of a person, not at their centre. For
// each compact cluster, estimate a stable surface point and push it away from
// the sensor by one body radius. Long clusters (usually walls) and isolated
// beams are dropped before association.

#ifndef SOCIAL_NAV_TRACKING_CPP__LIDAR_CLUSTERING_HPP_
#define SOCIAL_NAV_TRACKING_CPP__LIDAR_CLUSTERING_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace social_nav_tracking_cpp
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// Circle (x, y, radius) in the `map` frame treated as static.
struct ExclusionZone
{
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
};

/// Static map with precomputed distance to the nearest obstacle.
/// `grid` is the raw OccupancyGrid data (-1 = unknown), `clearance` the distance
/// transform times the resolution. Both are row-major, `width * height` long.
struct StaticMap
{
  std::vector<int8_t> grid;
  std::vector<double> clearance;
  std::size_t width = 0;
  std::size_t height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
};

/// Groups consecutive beams into estimated person centres.
/// `scan_indices` are the ORIGINAL beam indices in the LaserScan -- consecutive
/// is defined by beam order, not by spatial distance.
std::vector<Point2D> clusterLidarPersonCenters(
  const std::vector<Point2D> & points,
  const std::vector<int64_t> & scan_indices,
  const Point2D & sensor_origin,
  int min_points,
  double max_point_gap,
  double max_diameter,
  double person_radius);

/// Merges cluster centres closer than `merge_distance` into one centre.
///
/// At lidar height a walking person produces two separate clusters (the legs),
/// 0.3-0.6 m apart. Each cluster would become a track, and a lost leg track is
/// left behind on the person's path as a ghost the robot brakes for.
///
/// The threshold must be smaller than the distance between two people standing
/// close together, so two people are not merged into one.
/// merge_distance <= 0 does nothing.
std::vector<Point2D> mergeNearbyCenters(
  const std::vector<Point2D> & centers,
  double merge_distance);

/// Returns the INDICES of the centres kept after the static-map check.
/// Indices (rather than coordinates) avoid matching results back by exact
/// float comparison, which could in theory keep a rejected centre that has the
/// same coordinates as a kept one.
std::vector<std::size_t> filterByStaticMapIndices(
  const std::vector<Point2D> & centers,
  const StaticMap & map,
  double rejection_radius);

/// Same as above, for manually configured circular exclusion zones.
std::vector<std::size_t> filterByExclusionZonesIndices(
  const std::vector<Point2D> & centers,
  const std::vector<ExclusionZone> & zones);

}  // namespace social_nav_tracking_cpp

#endif  // SOCIAL_NAV_TRACKING_CPP__LIDAR_CLUSTERING_HPP_
