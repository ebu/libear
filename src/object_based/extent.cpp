#include "extent.hpp"

#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <utility>
#include "../common/geom.hpp"
#include "../common/helpers/eigen_helpers.hpp"

const double PI = boost::math::constants::pi<double>();

namespace ear {

  /** @brief Normalise position or return {0,1,0}.
   *
   * @param position  Position to normalise.
   *
   * @returns normalised position
   */
  Eigen::Vector3d safeNormPosition(Eigen::Vector3d position) {
    double norm = position.norm();
    if (norm < 1e-10) {
      return Eigen::Vector3d{0.0, 1.0, 0.0};
    } else {
      return position / norm;
    }
  }

  double extentMod(double extent, double distance) {
    double minSize = 0.2;
    double size = interp(extent, Eigen::Vector2d(0.0, 360.0),
                         Eigen::Vector2d(minSize, 1.0));
    double extent1 = 4.0 * degrees(atan2(size, 1.0));
    return interp(4.0 * degrees(atan2(size, distance)),
                  Eigen::Vector3d(0.0, extent1, 360.0),
                  Eigen::Vector3d(0.0, extent, 360.0));
  }

  Eigen::Matrix3d calcBasis(Eigen::Vector3d position) {
    position = safeNormPosition(position);
    double az = azimuth(position);
    double el = elevation(position);

    // points near the poles have indeterminate azimuth; assume 0
    if (abs(el) > (90.0 - 1e-5)) {
      az = 0.0;
    }
    return localCoordinateSystem(az, el);
  }

  Eigen::Vector3d cartOnBasis(Eigen::Matrix3d basis, double azimuth,
                              double elevation) {
    Eigen::RowVector3d cartPosRel{sin(azimuth) * cos(elevation),
                                  cos(azimuth) * cos(elevation),
                                  sin(elevation)};
    return cartPosRel * basis;
  }

  std::pair<double, double> azimuthElevationOnBasis(
      Eigen::Matrix3d basis, Eigen::RowVector3d position) {
    // project onto each basis, and clip components to keep asin happy
    Eigen::Vector3d components =
        (position * basis.transpose()).cwiseMin(1.0).cwiseMax(-1.0);

    double azimuth = atan2(components(0), components(1));
    double elevation = asin(components(2));

    return std::make_pair(azimuth, elevation);
  }

  WeightingFunction::WeightingFunction(Eigen::Vector3d position, double width,
                                       double height) {
    _width = radians(width) / 2;
    _height = radians(height) / 2;

    // basis vectors to rotate the vsource positions towards position
    Eigen::Matrix3d basises = calcBasis(position);

    _circleRadius = std::min(_width, _height);

    // Flip the width and the height such that it is always wider than it is
    // high from here in.
    if (_height > _width) {
      std::swap(_height, _width);
      _flippedBasis = basises.colwise().reverse();
    } else {
      _flippedBasis = basises;
    }

    // modify the width to make it meet at the back.
    double widthFull = PI + _height;
    // interpolate to this from a width of pi/2 to pi
    double widthMod = interp(_width, Eigen::Vector3d{0.0, PI / 2.0, PI},
                             Eigen::Vector3d{0.0, PI / 2.0, widthFull});
    // apply this fully for a height of less than pi/4; tail off until pi/2
    _width = interp(_height, Eigen::Vector4d{0, PI / 4.0, PI / 2.0, PI},  //
                    Eigen::Vector4d{widthMod, widthMod, _width, _width});  //

    // angle of the circle centres from the source position; width is to the
    // end of the rectangle.
    _circlePos = _width - _circleRadius;

    // Cartesian circle centres
    _circlePositions << cartOnBasis(_flippedBasis, -_circlePos, 0.0),
        cartOnBasis(_flippedBasis, _circlePos, 0.0);
  }

  double WeightingFunction::operator()(Eigen::Vector3d position) const {
    // Flipped azimuths and elevations; the straight edges are always along
    // azimuth lines.
    double azimuth, elevation;
    std::tie(azimuth, elevation) =
        azimuthElevationOnBasis(_flippedBasis, position);

    // The distance is the angle away from the defined shape; 0 or negative is
    // inside.
    double distance = 0.0;

    // for the straight lines
    if (abs(azimuth) <= _circlePos) {
      distance = abs(elevation) - _circleRadius;
    } else {
      // distance from the closest circle centre
      size_t nearest_circle = azimuth < 0 ? 0 : 1;
      double angle =
          position.transpose() * _circlePositions.col(nearest_circle);
      double circleDistance = acos(boost::algorithm::clamp(angle, -1.0, 1.0));
      distance = circleDistance - _circleRadius;
    }
    // fade the weight from one to zero over fadeWidth
    return interp(distance, Eigen::Vector2d{0.0, radians(_fadeWidth)},
                  Eigen::Vector2d{1.0, 0.0});
  }

  SpreadingPanner::SpreadingPanner(std::shared_ptr<PointSourcePanner> psp,
                                   int nRows)
      : _psp(psp), _nRows(nRows) {
    _panningPositions = _generatePanningPositionsEven();
    _panningPositionsResults = _generatePanningPositionsResults();
  }

  Eigen::VectorXd SpreadingPanner::panningValuesForWeight(
      const WeightingFunction& weightFunc) {
    Eigen::VectorXd weights(_panningPositions.rows());
    for (int i = 0; i < _panningPositions.rows(); ++i) {
      weights(i) = weightFunc(_panningPositions.row(i));
    }
    Eigen::VectorXd totalPv = weights.transpose() * _panningPositionsResults;
    return totalPv / totalPv.norm();
  }

  Eigen::MatrixXd SpreadingPanner::_generatePanningPositionsEven() {
    Eigen::VectorXd elevations =
        Eigen::VectorXd::LinSpaced(_nRows, -90.0, 90.0);
    Eigen::MatrixXd positions(0, 3);

    for (double el : elevations) {
      double radius = cos(radians(el));
      double perimiter = 2 * PI * radius;
      double perimiter_centre = 2 * PI;

      int nPoints = static_cast<int>(
          std::round((perimiter / perimiter_centre) * 2 * (_nRows - 1)));
      if (nPoints == 0) {
        nPoints = 1;
      }
      Eigen::VectorXd azimuths =
          Eigen::VectorXd::LinSpaced(nPoints + 1, 0.0, 360.0);
      for (int i = 0; i < azimuths.size() - 1; ++i) {
        double az = azimuths(i);
        positions.conservativeResize(positions.rows() + 1, Eigen::NoChange);
        positions.row(positions.rows() - 1) = cart(az, el, 1.0);
      }
    }
    return positions;
  }

  Eigen::MatrixXd SpreadingPanner::_generatePanningPositionsResults() {
    Eigen::MatrixXd results(_panningPositions.rows(),
                            _psp->numberOfOutputChannels());
    for (int i = 0; i < _panningPositions.rows(); ++i) {
      results.row(i) = _psp->handle(_panningPositions.row(i)).get();
    }
    return results;
  }

  PolarExtentPanner::PolarExtentPanner(std::shared_ptr<PointSourcePanner> psp)
      : _psp(psp), _spreadingPanner(SpreadingPanner(psp, _nRows)){};

  Eigen::VectorXd PolarExtentPanner::calcPvSpread(Eigen::Vector3d position,
                                                  double width, double height) {
    // When calculating the spread panning values the width and height are
    // set to at least fade_width. For sizes where any of the dimensions is
    // less than this, interpolate linearly between the point and spread
    // panning values.
    double ammount_spread =
        interp(std::max(width, height), Eigen::Vector2d(0.0, _fadeWidth),
               Eigen::Vector2d(0.0, 1.0));
    double ammountPoint = 1.0 - ammount_spread;
    Eigen::ArrayXd pv = Eigen::ArrayXd::Zero(_psp->numberOfOutputChannels());
    if (ammountPoint > 1e-10) {
      pv += ammountPoint * _psp->handle(position).get().array().square();
    }
    if (ammount_spread > 1e-10) {
      // minimum width and height as above
      width = std::max(width, _fadeWidth / 2.0);
      height = std::max(height, _fadeWidth / 2.0);

      WeightingFunction weightingFunction(position, width, height);
      Eigen::VectorXd panning_values =
          _spreadingPanner.panningValuesForWeight(weightingFunction);
      pv += ammount_spread * panning_values.array().square();
    }
    return pv.sqrt().matrix();
  }

  Eigen::VectorXd PolarExtentPanner::handle(Eigen::Vector3d position,
                                            double width, double height,
                                            double depth) {
    double distance = position.norm();

    if (depth != 0.0) {
      double distanceMin = distance - depth / 2.0;
      double distanceMax = distance + depth / 2.0;
      distanceMin = (distanceMin < 0) ? 0.0 : distanceMin;
      distanceMax = (distanceMax < 0) ? 0.0 : distanceMax;
      Eigen::VectorXd pvsMin =
          calcPvSpread(position, extentMod(width, distanceMin),
                       extentMod(height, distanceMin));
      Eigen::VectorXd pvsMax =
          calcPvSpread(position, extentMod(width, distanceMax),
                       extentMod(height, distanceMax));
      return ((pvsMin.array().square() + pvsMax.array().square()) / 2.0).sqrt();
    } else {
      Eigen::VectorXd pvs = calcPvSpread(position, extentMod(width, distance),
                                         extentMod(height, distance));
      return pvs;
    }
  }

}  // namespace ear
