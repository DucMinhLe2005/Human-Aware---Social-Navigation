// Compares the assignment with scipy.optimize.linear_sum_assignment.
//
// When several matchings have the SAME total cost (common, because the real
// cost matrices contain identical sentinel values) the optimum is not unique,
// so requiring the same matching would be wrong. Therefore:
//   - TOTAL cost: must be equal.
//   - Number of pairs: must be equal (= min(rows, cols)).
//   - Individual pairs: recorded for information only.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "social_nav_tracking_cpp/assignment.hpp"
#include "golden_reader.hpp"

using social_nav_tracking_cpp::CostMatrix;
using social_nav_tracking_cpp::solveAssignment;

TEST(Assignment, TotalCostMatchesScipy)
{
  const auto records = golden::read(std::string(GOLDEN_DIR) + "/lsap_calls.txt");
  ASSERT_FALSE(records.empty());

  std::size_t identical_pairs = 0;
  for (std::size_t i = 0; i < records.size(); ++i) {
    CostMatrix cost;
    cost.rows = static_cast<std::size_t>(records[i].integer("rows"));
    cost.cols = static_cast<std::size_t>(records[i].integer("cols"));
    cost.data = records[i].doubles("cost");
    ASSERT_EQ(cost.data.size(), cost.rows * cost.cols) << "case " << i;

    const auto expected_rows = records[i].ints("row_ind");
    const auto expected_cols = records[i].ints("col_ind");
    const double expected_total = records[i].scalar("total");

    const auto matches = solveAssignment(cost);

    ASSERT_EQ(matches.size(), expected_rows.size())
      << "case " << i << ": number of pairs differs from scipy";

    double total = 0.0;
    for (const auto & match : matches) {
      total += cost.at(match.first, match.second);
    }
    // Summation order may differ by a few ULP; 1e-12 is far below any decision this
    // value takes part in and still catches structural errors.
    EXPECT_NEAR(total, expected_total, 1e-12) << "case " << i << ": total cost differs from scipy";

    bool same = true;
    for (std::size_t k = 0; k < matches.size(); ++k) {
      if (static_cast<int>(matches[k].first) != expected_rows[k] ||
        static_cast<int>(matches[k].second) != expected_cols[k])
      {
        same = false;
        break;
      }
    }
    if (same) {
      ++identical_pairs;
    }
  }

  // Not a pass/fail criterion -- reports how often the two implementations pick
  // different matchings on ties.
  std::cout << "[info] identical pairs with scipy: " << identical_pairs << "/"
            << records.size() << " ca" << std::endl;
}

TEST(Assignment, EmptyMatrixReturnsEmpty)
{
  CostMatrix empty;
  EXPECT_TRUE(solveAssignment(empty).empty());

  CostMatrix no_cols;
  no_cols.rows = 3;
  no_cols.cols = 0;
  EXPECT_TRUE(solveAssignment(no_cols).empty());
}

TEST(Assignment, PicksTheObviousBestPairs)
{
  // Strongly dominant diagonal: no ties, the result must be unique.
  CostMatrix cost;
  cost.rows = 3;
  cost.cols = 3;
  cost.data = {
    0.1, 5.0, 5.0,
    5.0, 0.2, 5.0,
    5.0, 5.0, 0.3};
  const auto matches = solveAssignment(cost);
  ASSERT_EQ(matches.size(), 3u);
  EXPECT_EQ(matches[0].first, 0u);
  EXPECT_EQ(matches[0].second, 0u);
  EXPECT_EQ(matches[1].first, 1u);
  EXPECT_EQ(matches[1].second, 1u);
  EXPECT_EQ(matches[2].first, 2u);
  EXPECT_EQ(matches[2].second, 2u);
}

TEST(Assignment, MoreRowsThanColsMatchesColCount)
{
  CostMatrix cost;
  cost.rows = 4;
  cost.cols = 2;
  cost.data = {
    1.0, 9.0,
    9.0, 1.0,
    2.0, 2.0,
    3.0, 3.0};
  const auto matches = solveAssignment(cost);
  ASSERT_EQ(matches.size(), 2u);
  double total = 0.0;
  for (const auto & match : matches) {
    EXPECT_LT(match.first, 4u);
    EXPECT_LT(match.second, 2u);
    total += cost.at(match.first, match.second);
  }
  EXPECT_DOUBLE_EQ(total, 2.0);
}
