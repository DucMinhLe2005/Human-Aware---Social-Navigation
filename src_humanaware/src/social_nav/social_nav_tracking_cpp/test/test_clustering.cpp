// Compares lidar clustering and the two static filters with the Python
// implementation itself.
//
// Golden data is produced by loading human_tracking.py and calling the real
// functions (tools/dump_golden.py). Cluster centres must match bit for bit: a
// small median or percentile difference would shift the person by centimetres.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "social_nav_tracking_cpp/lidar_clustering.hpp"
#include "golden_reader.hpp"

using social_nav_tracking_cpp::ExclusionZone;
using social_nav_tracking_cpp::Point2D;
using social_nav_tracking_cpp::StaticMap;
using social_nav_tracking_cpp::clusterLidarPersonCenters;
using social_nav_tracking_cpp::filterByExclusionZonesIndices;
using social_nav_tracking_cpp::filterByStaticMapIndices;

namespace
{

std::vector<Point2D> toPoints(const std::vector<double> & flat)
{
  std::vector<Point2D> points;
  points.reserve(flat.size() / 2);
  for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
    points.push_back(Point2D{flat[i], flat[i + 1]});
  }
  return points;
}

}  // namespace

TEST(Clustering, ClusterCentresMatchPython)
{
  const auto records = golden::read(std::string(GOLDEN_DIR) + "/cluster_calls.txt");
  ASSERT_FALSE(records.empty());

  std::size_t total_centers = 0;
  for (std::size_t i = 0; i < records.size(); ++i) {
    const auto points = toPoints(records[i].doubles("points"));
    const auto raw_indices = records[i].ints("indices");
    std::vector<int64_t> indices(raw_indices.begin(), raw_indices.end());
    const auto sensor_flat = records[i].doubles("sensor");
    ASSERT_EQ(sensor_flat.size(), 2u);
    const Point2D sensor{sensor_flat[0], sensor_flat[1]};

    const auto actual = clusterLidarPersonCenters(
      points, indices, sensor,
      records[i].integer("min_points"),
      records[i].scalar("max_point_gap"),
      records[i].scalar("max_diameter"),
      records[i].scalar("person_radius"));

    const std::size_t expected_count =
      static_cast<std::size_t>(records[i].integer("n_centers"));
    ASSERT_EQ(actual.size(), expected_count)
      << "case " << i << ": cluster count differs from Python";
    if (expected_count == 0) {
      continue;
    }

    const auto expected = toPoints(records[i].doubles("centers"));
    ASSERT_EQ(expected.size(), expected_count);
    for (std::size_t k = 0; k < expected_count; ++k) {
      EXPECT_DOUBLE_EQ(actual[k].x, expected[k].x) << "case " << i << ", cluster " << k;
      EXPECT_DOUBLE_EQ(actual[k].y, expected[k].y) << "case " << i << ", cluster " << k;
    }
    total_centers += expected_count;
  }
  // If the golden data produced no cluster at all, the comparison above means nothing.
  EXPECT_GT(total_centers, 50u);
}

TEST(Clustering, StaticMapFilterMatchesPython)
{
  const auto records = golden::read(std::string(GOLDEN_DIR) + "/static_filter.txt");
  ASSERT_FALSE(records.empty());

  for (std::size_t i = 0; i < records.size(); ++i) {
    StaticMap map;
    map.width = static_cast<std::size_t>(records[i].integer("width"));
    map.height = static_cast<std::size_t>(records[i].integer("height"));
    map.resolution = records[i].scalar("resolution");
    map.origin_x = records[i].scalar("origin_x");
    map.origin_y = records[i].scalar("origin_y");
    map.origin_yaw = records[i].scalar("origin_yaw");
    const auto grid_values = records[i].ints("grid");
    map.grid.assign(grid_values.begin(), grid_values.end());
    map.clearance = records[i].doubles("clearance");

    const auto centers = toPoints(records[i].doubles("centers"));
    const auto kept_indices = filterByStaticMapIndices(
      centers, map, records[i].scalar("rejection_radius"));

    const std::size_t expected_count = static_cast<std::size_t>(records[i].integer("n_kept"));
    ASSERT_EQ(kept_indices.size(), expected_count) << "case " << i;
    if (expected_count == 0) {
      continue;
    }
    const auto expected = toPoints(records[i].doubles("kept"));
    for (std::size_t k = 0; k < expected_count; ++k) {
      EXPECT_DOUBLE_EQ(centers[kept_indices[k]].x, expected[k].x) << "case " << i;
      EXPECT_DOUBLE_EQ(centers[kept_indices[k]].y, expected[k].y) << "case " << i;
    }
  }
}

TEST(Clustering, ExclusionZoneFilterMatchesPython)
{
  const auto records = golden::read(std::string(GOLDEN_DIR) + "/exclusion_zones.txt");
  ASSERT_FALSE(records.empty());

  for (std::size_t i = 0; i < records.size(); ++i) {
    std::vector<ExclusionZone> zones;
    if (records[i].integer("n_zones") > 0) {
      const auto flat = records[i].doubles("zones");
      for (std::size_t k = 0; k + 2 < flat.size(); k += 3) {
        zones.push_back(ExclusionZone{flat[k], flat[k + 1], flat[k + 2]});
      }
    }

    const auto centers = toPoints(records[i].doubles("centers"));
    const auto kept_indices = filterByExclusionZonesIndices(centers, zones);

    const std::size_t expected_count = static_cast<std::size_t>(records[i].integer("n_kept"));
    ASSERT_EQ(kept_indices.size(), expected_count) << "case " << i;
    if (expected_count == 0) {
      continue;
    }
    const auto expected = toPoints(records[i].doubles("kept"));
    for (std::size_t k = 0; k < expected_count; ++k) {
      EXPECT_DOUBLE_EQ(centers[kept_indices[k]].x, expected[k].x) << "case " << i;
      EXPECT_DOUBLE_EQ(centers[kept_indices[k]].y, expected[k].y) << "case " << i;
    }
  }
}

TEST(Clustering, WideClusterRejectedAsWall)
{
  // A wall: 20 points spread over 2.4 m, above max_diameter -> all dropped.
  std::vector<Point2D> points;
  std::vector<int64_t> indices;
  for (int i = 0; i < 20; ++i) {
    points.push_back(Point2D{-1.2 + i * 0.12, 2.0});
    indices.push_back(i);
  }
  const auto centers = clusterLidarPersonCenters(
    points, indices, Point2D{0.0, 0.0}, 3, 0.15, 0.75, 0.28);
  EXPECT_TRUE(centers.empty());
}

TEST(Clustering, AllowsOneBadBeamInsideAPerson)
{
  // A gap of EXACTLY 2 beam indices still counts as one person.
  std::vector<Point2D> points;
  std::vector<int64_t> indices;
  const int64_t beam_indices[] = {10, 11, 13, 14};
  for (int i = 0; i < 4; ++i) {
    points.push_back(Point2D{0.02 * i, 2.0});
    indices.push_back(beam_indices[i]);
  }
  const auto centers = clusterLidarPersonCenters(
    points, indices, Point2D{0.0, 0.0}, 3, 0.15, 0.75, 0.28);
  EXPECT_EQ(centers.size(), 1u);

  // A 3-index gap splits it; each half is below min_points -> no cluster left.
  indices = {10, 11, 15, 16};
  const auto split = clusterLidarPersonCenters(
    points, indices, Point2D{0.0, 0.0}, 3, 0.15, 0.75, 0.28);
  EXPECT_TRUE(split.empty());
}
