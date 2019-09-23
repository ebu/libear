#pragma once
#include <boost/variant.hpp>
#include "common_types.hpp"
#include "export.hpp"

namespace ear {

  struct EAR_EXPORT CartesianScreen {
    double aspectRatio;
    CartesianPosition centrePosition;
    double widthX;
  };

  struct EAR_EXPORT PolarScreen {
    double aspectRatio;
    PolarPosition centrePosition;
    double widthAzimuth;
  };

  using Screen = boost::variant<PolarScreen, CartesianScreen>;

  Screen EAR_EXPORT getDefaultScreen();

}  // namespace ear
