#pragma once
#include <boost/variant.hpp>
#include <vector>
#include "export.hpp"

namespace ear {

  struct EAR_EXPORT CartesianPosition {
    CartesianPosition(double X = 0.0, double Y = 0.0, double Z = 0.0)
        : X(X), Y(Y), Z(Z){};
    double X;
    double Y;
    double Z;
  };

  struct EAR_EXPORT PolarPosition {
    PolarPosition(double azimuth = 0.0, double elevation = 0.0,
                  double distance = 1.0)
        : azimuth(azimuth), elevation(elevation), distance(distance){};
    double azimuth;
    double elevation;
    double distance;
  };

  using Position = boost::variant<CartesianPosition, PolarPosition>;

}  // namespace ear
