#include "social_nav_tracking_cpp/distance_transform.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace social_nav_tracking_cpp
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// 1-D distance transform of Felzenszwalb & Huttenlocher (2012), O(n):
// D(p) = min_q ( f[q] + (p-q)^2 ) via the lower envelope of parabolas. `f` in,
// `d` out; scratch arrays `v` / `z` are preallocated by the caller, because this
// runs width + height times per map.
void transform1D(
  const std::vector<double> & f,
  std::vector<double> & d,
  std::vector<std::size_t> & v,
  std::vector<double> & z,
  std::size_t n)
{
  // Skip positions with f = +inf (not sources). The envelope must start at the
  // first FINITE position: starting with a parabola at infinity would make the
  // intersection inf - inf = NaN, silently wrong.
  std::size_t first_finite = n;
  for (std::size_t i = 0; i < n; ++i) {
    if (f[i] != kInfinity) {
      first_finite = i;
      break;
    }
  }
  if (first_finite == n) {
    std::fill(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(n), kInfinity);
    return;
  }

  std::size_t k = 0;
  v[0] = first_finite;
  z[0] = -kInfinity;
  z[1] = kInfinity;

  for (std::size_t q = first_finite + 1; q < n; ++q) {
    const double fq = f[q];
    if (fq == kInfinity) {
      continue;
    }
    const double qd = static_cast<double>(q);
    double s = 0.0;
    while (true) {
      const double vd = static_cast<double>(v[k]);
      s = ((fq + qd * qd) - (f[v[k]] + vd * vd)) / (2.0 * qd - 2.0 * vd);
      if (s <= z[k] && k > 0) {
        --k;
        continue;
      }
      break;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInfinity;
  }

  std::size_t j = 0;
  for (std::size_t q = 0; q < n; ++q) {
    while (z[j + 1] < static_cast<double>(q)) {
      ++j;
    }
    const double diff = static_cast<double>(q) - static_cast<double>(v[j]);
    d[q] = diff * diff + f[v[j]];
  }
}

}  // namespace

std::vector<double> exactEuclideanDistanceTransform(
  const std::vector<char> & obstacles,
  std::size_t width,
  std::size_t height)
{
  std::vector<double> result(width * height, kInfinity);
  if (width == 0 || height == 0 || obstacles.size() != width * height) {
    return result;
  }

  bool any_obstacle = false;
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    if (obstacles[i]) {
      result[i] = 0.0;
      any_obstacle = true;
    }
  }
  if (!any_obstacle) {
    // No obstacle: every cell is +inf, not a large finite number (as in Python).
    return result;
  }

  const std::size_t longest = std::max(width, height);
  std::vector<double> f(longest);
  std::vector<double> d(longest);
  std::vector<std::size_t> v(longest + 1);
  std::vector<double> z(longest + 2);

  // Column pass then row pass: the transform is separable, so the result is the
  // EXACT squared Euclidean distance, not a chamfer approximation.
  for (std::size_t x = 0; x < width; ++x) {
    for (std::size_t y = 0; y < height; ++y) {
      f[y] = result[y * width + x];
    }
    transform1D(f, d, v, z, height);
    for (std::size_t y = 0; y < height; ++y) {
      result[y * width + x] = d[y];
    }
  }

  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      f[x] = result[y * width + x];
    }
    transform1D(f, d, v, z, width);
    for (std::size_t x = 0; x < width; ++x) {
      result[y * width + x] = std::sqrt(d[x]);
    }
  }

  return result;
}

}  // namespace social_nav_tracking_cpp
