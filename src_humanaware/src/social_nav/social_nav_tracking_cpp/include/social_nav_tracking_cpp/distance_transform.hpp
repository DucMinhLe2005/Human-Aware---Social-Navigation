// EXACT Euclidean distance transform, replacing
// scipy.ndimage.distance_transform_edt from the Python tracker.
//
// Computes, for every static-map cell, the distance to the nearest obstacle.
// The value feeds `clearance > lidar_static_rejection_radius` (0.40 m), which
// decides whether a lidar cluster is a PERSON or a WALL.
//
// Not cv::distanceTransform: OpenCV returns float32, whose error can flip the
// comparison for cells near the boundary, and it would pull OpenCV in for one
// function. Felzenszwalb-Huttenlocher computes squared distances in integers
// before the square root, so the result matches scipy bit for bit.

#ifndef SOCIAL_NAV_TRACKING_CPP__DISTANCE_TRANSFORM_HPP_
#define SOCIAL_NAV_TRACKING_CPP__DISTANCE_TRANSFORM_HPP_

#include <cstddef>
#include <vector>

namespace social_nav_tracking_cpp
{

/// Euclidean distance (in cells) from each cell to the nearest `true` cell of
/// `obstacles`; obstacle cells get 0. With NO obstacle, every cell is `+inf`.
///
/// `obstacles` is row-major and must be `width * height` long.
std::vector<double> exactEuclideanDistanceTransform(
  const std::vector<char> & obstacles,
  std::size_t width,
  std::size_t height);

}  // namespace social_nav_tracking_cpp

#endif  // SOCIAL_NAV_TRACKING_CPP__DISTANCE_TRANSFORM_HPP_
