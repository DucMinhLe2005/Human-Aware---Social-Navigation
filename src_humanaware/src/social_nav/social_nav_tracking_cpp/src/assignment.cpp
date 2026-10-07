#include "social_nav_tracking_cpp/assignment.hpp"

#include <algorithm>
#include <limits>

namespace social_nav_tracking_cpp
{
namespace
{

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// Hungarian algorithm, shortest augmenting path variant with potentials
// (u, v); requires rows <= cols. Classic 1-indexed formulation: `p[j]` is the
// row currently holding column j (0 = free). O(n^2 * m).
std::vector<std::size_t> solveRowsNotMoreThanCols(const CostMatrix & cost)
{
  const std::size_t n = cost.rows;
  const std::size_t m = cost.cols;

  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(m + 1, 0.0);
  std::vector<std::size_t> p(m + 1, 0);     // p[j] = row (1-indexed) holding column j
  std::vector<std::size_t> way(m + 1, 0);   // previous column on the augmenting path

  for (std::size_t i = 1; i <= n; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::vector<double> min_value(m + 1, kInfinity);
    std::vector<char> used(m + 1, 0);

    do {
      used[j0] = 1;
      const std::size_t i0 = p[j0];
      double delta = kInfinity;
      std::size_t j1 = 0;

      for (std::size_t j = 1; j <= m; ++j) {
        if (used[j]) {
          continue;
        }
        const double current = cost.at(i0 - 1, j - 1) - u[i0] - v[j];
        if (current < min_value[j]) {
          min_value[j] = current;
          way[j] = j0;
        }
        if (min_value[j] < delta) {
          delta = min_value[j];
          j1 = j;
        }
      }

      for (std::size_t j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          min_value[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  // Convert to row -> column form, 0-indexed; kSentinel = unassigned.
  std::vector<std::size_t> row_to_col(n, static_cast<std::size_t>(-1));
  for (std::size_t j = 1; j <= m; ++j) {
    if (p[j] != 0) {
      row_to_col[p[j] - 1] = j - 1;
    }
  }
  return row_to_col;
}

}  // namespace

std::vector<std::pair<std::size_t, std::size_t>> solveAssignment(const CostMatrix & cost)
{
  std::vector<std::pair<std::size_t, std::size_t>> matches;
  if (cost.rows == 0 || cost.cols == 0) {
    return matches;
  }

  // The algorithm above requires rows <= cols; otherwise transpose and swap back.
  const bool transposed = cost.rows > cost.cols;
  CostMatrix work;
  if (transposed) {
    work.rows = cost.cols;
    work.cols = cost.rows;
    work.data.resize(work.rows * work.cols);
    for (std::size_t r = 0; r < cost.rows; ++r) {
      for (std::size_t c = 0; c < cost.cols; ++c) {
        work.at(c, r) = cost.at(r, c);
      }
    }
  } else {
    work = cost;
  }

  const std::vector<std::size_t> row_to_col = solveRowsNotMoreThanCols(work);
  matches.reserve(row_to_col.size());
  for (std::size_t r = 0; r < row_to_col.size(); ++r) {
    const std::size_t c = row_to_col[r];
    if (c == static_cast<std::size_t>(-1)) {
      continue;
    }
    if (transposed) {
      matches.emplace_back(c, r);
    } else {
      matches.emplace_back(r, c);
    }
  }
  std::sort(matches.begin(), matches.end());
  return matches;
}

}  // namespace social_nav_tracking_cpp
