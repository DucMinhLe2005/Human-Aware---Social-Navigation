// Compares median/percentile with numpy itself (golden data from tools/dump_golden.py).
//
// These functions decide the CENTRE of a lidar cluster considered a person. An
// error here raises nothing and prints nothing -- it only shifts the person by a
// few centimetres. Hence bit-for-bit equality.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "social_nav_tracking_cpp/numpy_compat.hpp"
#include "golden_reader.hpp"

using social_nav_tracking_cpp::median;
using social_nav_tracking_cpp::percentileLinear;

TEST(NumpyCompat, MatchesNumpyBitForBit)
{
  const auto records = golden::read(std::string(GOLDEN_DIR) + "/median_percentile.txt");
  ASSERT_FALSE(records.empty());

  std::size_t checked = 0;
  for (std::size_t i = 0; i < records.size(); ++i) {
    const auto values = records[i].doubles("values");
    const double expected_median = records[i].scalar("median");
    const double expected_p20 = records[i].scalar("p20");

    EXPECT_DOUBLE_EQ(median(values), expected_median) << "case " << i << ", n=" << values.size();
    EXPECT_DOUBLE_EQ(percentileLinear(values, 20.0), expected_p20)
      << "case " << i << ", n=" << values.size();
    ++checked;
  }
  EXPECT_GT(checked, 100u);
}

TEST(NumpyCompat, EvenLengthAveragesTheTwoMiddleValues)
{
  // The most common mistake: many median implementations take the lower element
  // instead of the mean.
  EXPECT_DOUBLE_EQ(median({1.0, 2.0, 3.0, 4.0}), 2.5);
  EXPECT_DOUBLE_EQ(median({4.0, 3.0, 2.0, 1.0}), 2.5);
  EXPECT_DOUBLE_EQ(median({1.0, 2.0, 3.0}), 2.0);
}

TEST(NumpyCompat, PercentileInterpolatesInsteadOfRounding)
{
  // np.percentile([0,1,2,3,4], 20) = 0.8 -- interpolated, not the first element.
  EXPECT_DOUBLE_EQ(percentileLinear({0.0, 1.0, 2.0, 3.0, 4.0}, 20.0), 0.8);
  EXPECT_DOUBLE_EQ(percentileLinear({5.0}, 20.0), 5.0);
  EXPECT_DOUBLE_EQ(percentileLinear({1.0, 2.0}, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(percentileLinear({1.0, 2.0}, 100.0), 2.0);
}

TEST(NumpyCompat, EmptyArrayDoesNotCrash)
{
  // Python never calls this with an empty array, but a C++ library must not have
  // undefined behaviour because of a careless caller.
  EXPECT_DOUBLE_EQ(median({}), 0.0);
  EXPECT_DOUBLE_EQ(percentileLinear({}, 20.0), 0.0);
}
