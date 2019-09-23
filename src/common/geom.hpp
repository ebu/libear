#pragma once
#include <Eigen/Core>
#include <boost/math/constants/constants.hpp>
#include "ear/common_types.hpp"
#include "ear/metadata.hpp"

namespace ear {

  template <typename T>
  inline T radians(T d) {
    return d * static_cast<T>(boost::math::constants::pi<double>() / 180.0);
  }

  template <typename T>
  inline T degrees(T r) {
    return r * static_cast<T>(180.0 / boost::math::constants::pi<double>());
  }

  /** @brief Assuming y is clockwise from x, increment y by 360 until it's not
   * less than x.
   *
   * @param x Start angle in degrees.
   * @param y End angle in degrees.
   *
   * @returns y shifted such that it represents the same angle but is greater
   *  than x.
   */
  inline double relativeAngle(double x, double y) {
    while (y - 360.0 >= x) {
      y -= 360.0;
    }
    while (y < x) {
      y += 360.0;
    }
    return y;
  }

  /** @brief Assuming end is clockwise from start, is the angle x inside
   * [start,end] within some tolerance?
   *
   * @param x Angle to test, in degrees
   * @param start Start angle of range, in degrees
   * @param end End angle of range, in degrees
   * @param tol Tolerance in degrees to check within
   */
  bool insideAngleRange(double x, double start, double end, double tol = 0.0);

  /** @brief Order the vertices of a convex, approximately planar polygon.
   *
   * @param vertices Vertices to order (n, 3)
   *
   * @returns Indices to put the vertices into the right order.
   */
  Eigen::VectorXi ngonVertexOrder(Eigen::MatrixXd vertices);

  double azimuth(Eigen::Vector3d position);
  double elevation(Eigen::Vector3d position);
  double distance(Eigen::Vector3d position);

  Eigen::Vector3d cart(double azimuth, double elevation, double distance);

  inline Eigen::RowVector3d cartT(double azimuth, double elevation,
                                  double distance) {
    return cart(azimuth, elevation, distance).transpose();
  }

  PolarPosition toPolarPosition(CartesianPosition position);
  CartesianPosition toCartesianPosition(PolarPosition position);
  Eigen::Vector3d toNormalisedVector3d(PolarPosition position);

  Eigen::Vector3d toCartesianVector3d(PolarPosition position);
  Eigen::Vector3d toCartesianVector3d(CartesianPosition position);
  Eigen::Vector3d toCartesianVector3d(SpeakerPosition position);
  Eigen::Vector3d toCartesianVector3d(PolarSpeakerPosition position);
  Eigen::Vector3d toCartesianVector3d(CartesianSpeakerPosition position);

  Eigen::MatrixXd toPositionsMatrix(
      const std::vector<PolarPosition>& positions);

  /** @brief Calc local coordinate system
   *
   * @param azimuth: ADM format azimuth
   * @param elevation: ADM format elevation
   *
   * @return Vectors pointing along x, y and z, rotated so that +y points at
   * cart(az, el, 1).
   */
  inline Eigen::Matrix3d localCoordinateSystem(double azimuth,
                                               double elevation) {
    Eigen::Matrix3d ret;
    ret << cartT(azimuth - 90.0, 0.0, 1.0),  //
        cartT(azimuth, elevation, 1.0),  //
        cartT(azimuth, elevation + 90.0, 1.0);
    return ret;
  }

}  // namespace ear
