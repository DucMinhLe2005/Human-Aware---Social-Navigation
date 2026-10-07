#include "social_nav_tracking_cpp/lidar_clustering.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "social_nav_tracking_cpp/numpy_compat.hpp"

namespace social_nav_tracking_cpp
{
namespace
{

bool isFinite2D(const Point2D & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

}  // namespace

std::vector<Point2D> clusterLidarPersonCenters(
  const std::vector<Point2D> & points,
  const std::vector<int64_t> & scan_indices,
  const Point2D & sensor_origin,
  int min_points,
  double max_point_gap,
  double max_diameter,
  double person_radius)
{
  std::vector<Point2D> centers;
  if (points.empty() || points.size() != scan_indices.size()) {
    return centers;
  }

  // Drop non-finite points.
  std::vector<Point2D> filtered_points;
  std::vector<int64_t> filtered_indices;
  filtered_points.reserve(points.size());
  filtered_indices.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (isFinite2D(points[i])) {
      filtered_points.push_back(points[i]);
      filtered_indices.push_back(scan_indices[i]);
    }
  }
  if (filtered_points.empty()) {
    return centers;
  }

  if (!isFinite2D(sensor_origin)) {
    return centers;
  }

  const std::size_t count = filtered_points.size();
  const int required_points = std::max(1, min_points);
  const double point_gap = std::max(1e-6, max_point_gap);
  const double diameter_limit = std::max(1e-6, max_diameter);
  const double radius = std::max(0.0, person_radius);

  // Sort by original beam index. The indices already come sorted from
  // np.nonzero in the Python path; stable_sort is the safe superset.
  std::vector<std::size_t> order(count);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(
    order.begin(), order.end(),
    [&filtered_indices](std::size_t a, std::size_t b) {
      return filtered_indices[a] < filtered_indices[b];
    });

  std::vector<Point2D> sorted_points(count);
  std::vector<int64_t> sorted_indices(count);
  for (std::size_t i = 0; i < count; ++i) {
    sorted_points[i] = filtered_points[order[i]];
    sorted_indices[i] = filtered_indices[order[i]];
  }

  // Split clusters: between consecutive beams more than 2 indices apart, or too
  // far apart in space. The 2-index tolerance lets ONE invalid beam sit inside a
  // person without joining two separate objects.
  std::vector<std::size_t> bounds;
  bounds.push_back(0);
  for (std::size_t i = 1; i < count; ++i) {
    const int64_t scan_gap = sorted_indices[i] - sorted_indices[i - 1];
    const double dx = sorted_points[i].x - sorted_points[i - 1].x;
    const double dy = sorted_points[i].y - sorted_points[i - 1].y;
    // np.linalg.norm on a 2-vector is sqrt(dx^2 + dy^2), NOT hypot; the two differ
    // in the last digit in about 17% of cases.
    const double spatial_gap = std::sqrt(dx * dx + dy * dy);
    if (scan_gap > 2 || spatial_gap > point_gap) {
      bounds.push_back(i);
    }
  }
  bounds.push_back(count);

  std::vector<double> ranges;
  std::vector<double> unit_x;
  std::vector<double> unit_y;

  for (std::size_t b = 0; b + 1 < bounds.size(); ++b) {
    const std::size_t start = bounds[b];
    const std::size_t end = bounds[b + 1];
    const std::size_t size = end - start;
    if (size < static_cast<std::size_t>(required_points)) {
      continue;
    }

    double min_x = sorted_points[start].x;
    double max_x = min_x;
    double min_y = sorted_points[start].y;
    double max_y = min_y;
    for (std::size_t i = start + 1; i < end; ++i) {
      min_x = std::min(min_x, sorted_points[i].x);
      max_x = std::max(max_x, sorted_points[i].x);
      min_y = std::min(min_y, sorted_points[i].y);
      max_y = std::max(max_y, sorted_points[i].y);
    }
    // np.hypot here (unlike the sqrt above), as in the Python reference.
    const double cluster_diameter = std::hypot(max_x - min_x, max_y - min_y);
    if (cluster_diameter > diameter_limit) {
      continue;
    }

    ranges.clear();
    unit_x.clear();
    unit_y.clear();
    for (std::size_t i = start; i < end; ++i) {
      const double dx = sorted_points[i].x - sensor_origin.x;
      const double dy = sorted_points[i].y - sensor_origin.y;
      const double range = std::sqrt(dx * dx + dy * dy);
      if (range <= 1e-6) {
        continue;
      }
      ranges.push_back(range);
      unit_x.push_back(dx / range);
      unit_y.push_back(dy / range);
    }
    if (ranges.empty()) {
      continue;
    }

    // A single shortest beam is sensitive to range noise and beam jumps. Use the
    // PER-AXIS median of the unit vectors for the direction (not the geometric
    // median) and a low percentile for the range: keeps the visible front surface
    // while ignoring one abnormally short beam.
    double direction_x = median(unit_x);
    double direction_y = median(unit_y);
    const double direction_length = std::sqrt(
      direction_x * direction_x + direction_y * direction_y);
    if (direction_length <= 1e-6) {
      continue;
    }
    direction_x /= direction_length;
    direction_y /= direction_length;

    const double surface_range = percentileLinear(ranges, 20.0);
    Point2D center;
    center.x = sensor_origin.x + (surface_range + radius) * direction_x;
    center.y = sensor_origin.y + (surface_range + radius) * direction_y;
    centers.push_back(center);
  }

  return centers;
}

std::vector<Point2D> mergeNearbyCenters(
  const std::vector<Point2D> & centers, double merge_distance)
{
  if (merge_distance <= 0.0 || centers.size() < 2) {
    return centers;
  }
  const double threshold_sq = merge_distance * merge_distance;
  std::vector<bool> da_gop(centers.size(), false);
  std::vector<Point2D> result;
  result.reserve(centers.size());

  for (std::size_t i = 0; i < centers.size(); ++i) {
    if (da_gop[i] || !isFinite2D(centers[i])) {
      continue;
    }
    // Transitive merge: if A is near B and B near C, all three are one person.
    // One pass is enough for two legs.
    double tong_x = centers[i].x;
    double tong_y = centers[i].y;
    std::size_t dem = 1;
    da_gop[i] = true;
    for (std::size_t j = i + 1; j < centers.size(); ++j) {
      if (da_gop[j] || !isFinite2D(centers[j])) {
        continue;
      }
      const double dx = centers[j].x - centers[i].x;
      const double dy = centers[j].y - centers[i].y;
      if (dx * dx + dy * dy <= threshold_sq) {
        tong_x += centers[j].x;
        tong_y += centers[j].y;
        dem += 1;
        da_gop[j] = true;
      }
    }
    result.push_back(
      Point2D{tong_x / static_cast<double>(dem), tong_y / static_cast<double>(dem)});
  }
  return result;
}

std::vector<std::size_t> filterByStaticMapIndices(
  const std::vector<Point2D> & centers,
  const StaticMap & map,
  double rejection_radius)
{
  std::vector<std::size_t> kept;
  if (centers.empty()) {
    return kept;
  }
  if (map.width == 0 || map.height == 0 ||
    map.grid.size() != map.width * map.height ||
    map.clearance.size() != map.grid.size() ||
    !std::isfinite(map.resolution) || map.resolution <= 0.0)
  {
    return kept;
  }

  const double cos_yaw = std::cos(map.origin_yaw);
  const double sin_yaw = std::sin(map.origin_yaw);
  const double min_clearance = std::max(0.0, rejection_radius);

  for (std::size_t i = 0; i < centers.size(); ++i) {
    if (!isFinite2D(centers[i])) {
      continue;
    }
    const double dx = centers[i].x - map.origin_x;
    const double dy = centers[i].y - map.origin_y;
    const double map_x = cos_yaw * dx + sin_yaw * dy;
    const double map_y = -sin_yaw * dx + cos_yaw * dy;
    const double cell_x_real = std::floor(map_x / map.resolution);
    const double cell_y_real = std::floor(map_y / map.resolution);
    if (cell_x_real < 0.0 || cell_y_real < 0.0 ||
      cell_x_real >= static_cast<double>(map.width) ||
      cell_y_real >= static_cast<double>(map.height))
    {
      continue;
    }
    const std::size_t cell_x = static_cast<std::size_t>(cell_x_real);
    const std::size_t cell_y = static_cast<std::size_t>(cell_y_real);
    const std::size_t index = cell_y * map.width + cell_x;
    // grid >= 0: known cell (unknown is -1). clearance > threshold: far enough from obstacles.
    if (map.grid[index] >= 0 && map.clearance[index] > min_clearance) {
      kept.push_back(i);
    }
  }
  return kept;
}

std::vector<std::size_t> filterByExclusionZonesIndices(
  const std::vector<Point2D> & centers,
  const std::vector<ExclusionZone> & zones)
{
  std::vector<ExclusionZone> valid_zones;
  valid_zones.reserve(zones.size());
  for (const auto & zone : zones) {
    if (std::isfinite(zone.x) && std::isfinite(zone.y) && std::isfinite(zone.radius)) {
      ExclusionZone clamped = zone;
      clamped.radius = std::max(0.0, zone.radius);
      valid_zones.push_back(clamped);
    }
  }

  std::vector<std::size_t> kept;
  kept.reserve(centers.size());
  for (std::size_t i = 0; i < centers.size(); ++i) {
    if (!isFinite2D(centers[i])) {
      continue;
    }
    bool inside = false;
    for (const auto & zone : valid_zones) {
      if (std::hypot(centers[i].x - zone.x, centers[i].y - zone.y) <= zone.radius) {
        inside = true;
        break;
      }
    }
    if (!inside) {
      kept.push_back(i);
    }
  }
  return kept;
}

}  // namespace social_nav_tracking_cpp
