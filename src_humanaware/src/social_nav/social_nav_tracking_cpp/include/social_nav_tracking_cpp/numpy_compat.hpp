// numpy reductions used by the Python tracker, reimplemented with the exact
// numpy semantics.
//
// Both functions decide the CENTRE of a lidar cluster considered a person. A
// small deviation here causes no error and no warning -- it only shifts the
// person by a few centimetres, and the consequences (misplaced social zone,
// wrong association) show up several layers later. Hence a separate file and
// separate tests.

#ifndef SOCIAL_NAV_TRACKING_CPP__NUMPY_COMPAT_HPP_
#define SOCIAL_NAV_TRACKING_CPP__NUMPY_COMPAT_HPP_

#include <vector>

namespace social_nav_tracking_cpp
{

/// Median as `np.median`: odd length takes the middle element, even length the
/// MEAN of the two middle elements. `values` is copied and sorted internally.
/// An empty vector returns 0.0 (never called with an empty array).
double median(std::vector<double> values);

/// Percentile as `np.percentile(..., method='linear')` (numpy's default):
/// position `i = q/100 * (n - 1)`, then linear interpolation between the two
/// integer neighbours. `q` is in percent (0..100).
double percentileLinear(std::vector<double> values, double q);

}  // namespace social_nav_tracking_cpp

#endif  // SOCIAL_NAV_TRACKING_CPP__NUMPY_COMPAT_HPP_
