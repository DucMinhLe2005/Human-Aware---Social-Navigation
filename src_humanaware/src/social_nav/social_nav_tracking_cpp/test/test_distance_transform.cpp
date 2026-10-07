// Compares the distance transform with scipy.ndimage.distance_transform_edt.
//
// Bit-for-bit equality, no tolerance: both sides compute squared distances in
// integers and take a single square root, so the results must match exactly.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "social_nav_tracking_cpp/distance_transform.hpp"
#include "golden_reader.hpp"

using social_nav_tracking_cpp::exactEuclideanDistanceTransform;

TEST(DistanceTransform, MatchesScipy)
{
  const auto records = golden::read(std::string(GOLDEN_DIR) + "/edt_maps.txt");
  ASSERT_FALSE(records.empty());

  for (std::size_t i = 0; i < records.size(); ++i) {
    const std::size_t width = static_cast<std::size_t>(records[i].integer("width"));
    const std::size_t height = static_cast<std::size_t>(records[i].integer("height"));
    const auto raw = records[i].ints("obstacles");
    const auto expected = records[i].doubles("distance");

    ASSERT_EQ(raw.size(), width * height) << "case " << i;
    ASSERT_EQ(expected.size(), width * height) << "case " << i;

    std::vector<char> obstacles(raw.size());
    for (std::size_t k = 0; k < raw.size(); ++k) {
      obstacles[k] = static_cast<char>(raw[k] != 0);
    }

    const auto actual = exactEuclideanDistanceTransform(obstacles, width, height);
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t k = 0; k < expected.size(); ++k) {
      EXPECT_DOUBLE_EQ(actual[k], expected[k])
        << "case " << i << ", o " << k << " (" << (k % width) << "," << (k / width) << ")";
    }
  }
}

TEST(DistanceTransform, NoObstacleGivesInfinity)
{
  // No obstacle yields np.inf, not a large finite number -- this matters because
  // the value feeds `clearance > lidar_static_rejection_radius`.
  const std::vector<char> empty_map(6 * 5, 0);
  const auto result = exactEuclideanDistanceTransform(empty_map, 6, 5);
  ASSERT_EQ(result.size(), 30u);
  for (const double value : result) {
    EXPECT_TRUE(std::isinf(value));
    EXPECT_GT(value, 0.0);
  }
}

TEST(DistanceTransform, ObstacleCellHasZeroDistance)
{
  std::vector<char> obstacles(4 * 4, 0);
  obstacles[5] = 1;
  const auto result = exactEuclideanDistanceTransform(obstacles, 4, 4);
  EXPECT_DOUBLE_EQ(result[5], 0.0);
  EXPECT_DOUBLE_EQ(result[4], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 1.0);
  EXPECT_DOUBLE_EQ(result[0], std::sqrt(2.0));
}

TEST(DistanceTransform, WrongSizeReturnsInfinityInsteadOfCrashing)
{
  const std::vector<char> obstacles(5, 0);
  const auto result = exactEuclideanDistanceTransform(obstacles, 3, 3);
  ASSERT_EQ(result.size(), 9u);
  for (const double value : result) {
    EXPECT_TRUE(std::isinf(value));
  }
}
