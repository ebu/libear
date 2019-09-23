#pragma once
#include <Eigen/Core>
#include <boost/optional.hpp>
#include "ear/exceptions.hpp"
#include "ear/metadata.hpp"
#include "ear/screen.hpp"

namespace ear {
  class ScreenEdgeLockHandler {
   public:
    ScreenEdgeLockHandler(boost::optional<Screen> reproductionScreen)
        : _reproductionScreen(reproductionScreen){};

    std::pair<double, double> handleAzimuthElevation(
        double azimuth, double elevation, ScreenEdgeLock screenEdgeLock) {
      if (screenEdgeLock.horizontal || screenEdgeLock.vertical)
        throw not_implemented("screenEdgeLock");

      return std::make_pair(azimuth, elevation);
    }

    std::tuple<double, double, double> handleVector(
        Eigen::Vector3d pos, ScreenEdgeLock screenEdgeLock) {
      if (screenEdgeLock.horizontal || screenEdgeLock.vertical)
        throw not_implemented("screenEdgeLock");

      return std::make_tuple(pos(0), pos(1), pos(2));
    }

   private:
    boost::optional<Screen> _reproductionScreen;
  };
}  // namespace ear
