#pragma once
#include "ear/common_types.hpp"

namespace ear {
  inline bool operator==(const CartesianPosition& a,
                         const CartesianPosition& b) {
    return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
  }

  inline bool operator!=(const CartesianPosition& a,
                         const CartesianPosition& b) {
    return !(a == b);
  }

  inline bool operator==(const PolarPosition& a, const PolarPosition& b) {
    return a.azimuth == b.azimuth && a.elevation == b.elevation &&
           a.distance == b.distance;
  }

  inline bool operator!=(const PolarPosition& a, const PolarPosition& b) {
    return !(a == b);
  }
};  // namespace ear
