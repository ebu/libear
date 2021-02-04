#pragma once
#include <Eigen/Core>
#include "facets.hpp"

namespace ear {
  /// very simple convex hull implementation
  ///
  /// limitations:
  /// - doesn't deal with degenerate cases (less than 4 points, or all points
  ///   being coplanar)
  /// - doesn't deal with collinear points at all (we only use points a unit
  ///   distance from the origin, so this never occurs)
  /// - doesn't check for valid geometry, so the tolerance has to be high
  ///   enough
  std::vector<Facet> convex_hull(const std::vector<Eigen::Vector3d> &positions,
                                 double tolerance = 1e-5);
}  // namespace ear
