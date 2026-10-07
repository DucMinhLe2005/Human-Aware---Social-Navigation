// Globally optimal assignment (linear sum assignment), replacing
// scipy.optimize.linear_sum_assignment from the Python tracker.
//
// Implemented here instead of pulling in dlib/or-tools for a single function;
// problem sizes are around 10x10 (tracks x candidates).
//
// UNIQUENESS: when several assignments have the same total cost (common with
// symmetric points), the optimum is not unique and this implementation and
// scipy may return DIFFERENT, equally correct matchings. Tests therefore
// compare the TOTAL cost, not individual pairs -- see test/test_assignment.cpp.

#ifndef SOCIAL_NAV_TRACKING_CPP__ASSIGNMENT_HPP_
#define SOCIAL_NAV_TRACKING_CPP__ASSIGNMENT_HPP_

#include <cstddef>
#include <utility>
#include <vector>

namespace social_nav_tracking_cpp
{

/// Dense cost matrix, row-major.
struct CostMatrix
{
  std::size_t rows = 0;
  std::size_t cols = 0;
  std::vector<double> data;

  double at(std::size_t r, std::size_t c) const {return data[r * cols + c];}
  double & at(std::size_t r, std::size_t c) {return data[r * cols + c];}
};

/// Solves the linear assignment problem (Hungarian / Jonker-Volgenant shortest
/// augmenting path, O(n^2 m)) and returns the (row, col) pairs with minimum
/// TOTAL cost. Number of pairs = min(rows, cols), as in scipy.
///
/// An empty matrix in either dimension returns an empty list. Every cell must
/// be finite -- out-of-gate pairs use a large FINITE sentinel, not inf.
std::vector<std::pair<std::size_t, std::size_t>> solveAssignment(const CostMatrix & cost);

}  // namespace social_nav_tracking_cpp

#endif  // SOCIAL_NAV_TRACKING_CPP__ASSIGNMENT_HPP_
