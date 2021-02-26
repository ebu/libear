#include "extent.hpp"

#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <boost/make_unique.hpp>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <utility>
#include "../common/geom.hpp"
#include "../common/helpers/eigen_helpers.hpp"

#ifndef EAR_DISABLE_HTM
#include "extent_htm.hpp"
#endif

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

  AngleToWeight::AngleToWeight() {}
  AngleToWeight::AngleToWeight(double start_angle, double end_angle)
      // tolerance in end_angle is to make sure that if start_angle is less
      // than PI but end_angle isn't, we always use interpolation after
      // start_angle
      : cos_start_angle(start_angle < PI ? std::cos(start_angle) : -1.0),
        cos_end_angle(end_angle < PI ? std::cos(end_angle) : -(1.0 + 1e-6)),
        // sin is only used for less than PI/2
        sin_start_angle(start_angle < PI / 2 ? std::sin(start_angle) : 1.0),
        sin_end_angle(end_angle < PI / 2 ? std::sin(end_angle) : 1.0 + 1e-6),
        // between start and end angle, we want:
        // out = (angle - end_angle) / (start_angle - end_angle)
        // therefore the slope is:
        m(1.0 / (start_angle - end_angle)),
        // out = m * (angle - end_angle)
        // out = m * angle - m * end_angle
        // intercept is:
        c(-m * end_angle) {
    ear_assert(start_angle >= 0, "start angle should be +ve");
    ear_assert(end_angle >= 0, "end angle should be +ve");
    ear_assert(end_angle > start_angle, "end angle should be > start angle");
  }

  double AngleToWeight::from_cos(double cos_angle) const {
    if (cos_angle >= cos_start_angle)
      return 1.0;
    else if (cos_angle <= cos_end_angle)
      return 0.0;
    else
      return m * std::acos(cos_angle) + c;
  }

  double AngleToWeight::from_sin(double sin_angle) const {
    if (sin_angle <= sin_start_angle)
      return 1.0;
    else if (sin_angle >= sin_end_angle)
      return 0.0;
    else
      return m * std::asin(sin_angle) + c;
  }

  Eigen::Matrix3d calcBasis(Eigen::Vector3d position) {
    position = safeNormPosition(position);
    double az = azimuth(position);
    double el = elevation(position);

    // points near the poles have indeterminate azimuth; assume 0
    if (std::abs(el) > (90.0 - 1e-5)) {
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

  WeightingFunction::WeightingFunction(Eigen::Vector3d position, double width,
                                       double height) {
    _width = radians(width) / 2;
    _height = radians(height) / 2;

    // basis vectors to rotate the vsource positions towards position
    Eigen::Matrix3d basises = calcBasis(position);

    // Flip the width and the height such that it is always wider than it is
    // high from here in.
    if (_height > _width) {
      std::swap(_height, _width);
      // rotate rather than flipping x and y to preserve triangle winding
      Eigen::Matrix3d m;
      m << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;
      _flippedBasis = m * basises;
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

    right_circle_centre << sin(circlePos()), cos(circlePos());
    circle_test << -cos(circlePos()), sin(circlePos());

    angle_to_weight =
        AngleToWeight(circleRadius(), circleRadius() + radians(_fadeWidth));

    is_circular = (_width - _height) < 1e-6;

    weight_cb = is_circular ? weight_circle : weight_stadium;
  }

  double WeightingFunction::weight_circle(
      const WeightingFunction &self,
      const Eigen::Ref<const Eigen::RowVector3d> &position) {
    // simplified dot product assuming that circle_centre is {0, 1, 0}
    double dot = self._flippedBasis.row(1).dot(position);
    return self.angle_to_weight.from_cos(dot);
  }

  double WeightingFunction::weight_stadium(
      const WeightingFunction &self,
      const Eigen::Ref<const Eigen::RowVector3d> &position) {
    Eigen::RowVector3d position_t = self._flippedBasis * position.transpose();

    Eigen::RowVector3d position_t_right = position_t;
    position_t_right(0) = std::abs(position_t(0));

    // for the straight lines
    if (position_t_right.head<2>() * self.circle_test >= 0) {
      return self.angle_to_weight.from_sin(std::abs(position_t(2)));
    } else {
      double dot = self.right_circle_centre.dot(position_t_right.head<2>());

      return self.angle_to_weight.from_cos(dot);
    }
  }

  double WeightingFunction::operator()(
      const Eigen::Ref<const Eigen::RowVector3d> &position) const {
    return weight_cb(*this, position);
  }

  SpreadingPanner::SpreadingPanner(std::shared_ptr<PointSourcePanner> psp,
                                   int nRows)
      : SpreadingPannerBase(),
        _panningPositions(_generatePanningPositionsEven(nRows)),
        _panningPositionsResults(
            _generatePanningPositionsResults(psp, _panningPositions)) {}

  Eigen::VectorXd SpreadingPanner::panningValuesForWeight(
      const WeightingFunction &weightFunc) const {
    Eigen::VectorXd totalPv =
        Eigen::VectorXd::Zero(_panningPositionsResults.rows());
    for (int i = 0; i < _panningPositions.rows(); ++i) {
      double weight = weightFunc(_panningPositions.row(i));
      if (weight != 0.0)
        totalPv.noalias() += _panningPositionsResults.col(i) * weight;
    }
    totalPv /= totalPv.norm();
    return totalPv;
  }

  Eigen::MatrixXd SpreadingPannerBase::_generatePanningPositionsEven(
      int nRows) {
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(nRows, -90.0, 90.0);
    Eigen::MatrixXd positions(0, 3);

    for (double el : elevations) {
      double radius = cos(radians(el));
      double perimiter = 2 * PI * radius;
      double perimiter_centre = 2 * PI;

      int nPoints = static_cast<int>(
          std::round((perimiter / perimiter_centre) * 2 * (nRows - 1)));
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

  Eigen::MatrixXd SpreadingPannerBase::_generatePanningPositionsResults(
      std::shared_ptr<PointSourcePanner> psp,
      const Eigen::Ref<const Eigen::MatrixXd> &positions) {
    Eigen::MatrixXd results(psp->numberOfOutputChannels(), positions.rows());
    for (int i = 0; i < positions.rows(); ++i) {
      results.col(i) = psp->handle(positions.row(i)).get();
    }
    return results;
  }

  const int PolarExtentPanner::nRowsDefault = 37;  // 5 degrees per row

  PolarExtentPanner::PolarExtentPanner(
      std::shared_ptr<PointSourcePanner> psp,
      std::unique_ptr<SpreadingPannerBase> spreadingPanner)
      : _psp(psp), _spreadingPanner(std::move(spreadingPanner)) {}

  static std::unique_ptr<SpreadingPannerBase> make_default_spreading_panner(
      std::shared_ptr<PointSourcePanner> psp) {
#ifdef EAR_DISABLE_HTM
    return boost::make_unique<SpreadingPanner>(psp,
                                               PolarExtentPanner::nRowsDefault);
#else
    return boost::make_unique<SpreadingPannerHTM>(
        psp, PolarExtentPanner::nRowsDefault, 1);
#endif
  }

  PolarExtentPanner::PolarExtentPanner(std::shared_ptr<PointSourcePanner> psp)
      : PolarExtentPanner(psp, make_default_spreading_panner(psp)) {}

  Eigen::VectorXd PolarExtentPanner::calcPvSpread(Eigen::Vector3d position,
                                                  double width,
                                                  double height) const {
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
          _spreadingPanner->panningValuesForWeight(weightingFunction);
      pv += ammount_spread * panning_values.array().square();
    }
    return pv.sqrt().matrix();
  }

  Eigen::VectorXd PolarExtentPanner::handle(Eigen::Vector3d position,
                                            double width, double height,
                                            double depth) const {
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
