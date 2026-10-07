#include "social_nav_tracking_cpp/numpy_compat.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace social_nav_tracking_cpp
{

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t n = values.size();
  const std::size_t mid = n / 2;
  // nth_element suffices for odd lengths; even lengths also need the previous
  // element, taken as the max of the lower half -- still O(n), no full sort.
  std::nth_element(
    values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  const double upper = values[mid];
  if (n % 2 == 1) {
    return upper;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(mid);
  const double lower = *std::max_element(values.begin(), middle);
  return 0.5 * (lower + upper);
}

double percentileLinear(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  if (n == 1) {
    return values[0];
  }
  const double clamped_q = std::min(100.0, std::max(0.0, q));
  const double position = clamped_q / 100.0 * static_cast<double>(n - 1);
  const double floor_position = std::floor(position);
  const std::size_t lower_index = static_cast<std::size_t>(floor_position);
  if (lower_index + 1 >= n) {
    return values[n - 1];
  }
  const double fraction = position - floor_position;
  return values[lower_index] + fraction * (values[lower_index + 1] - values[lower_index]);
}

}  // namespace social_nav_tracking_cpp
