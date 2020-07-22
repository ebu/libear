#pragma once
#include <Eigen/Core>
#include <memory>
#include "common/point_source_panner.hpp"

namespace ear {
  namespace reference {

    class ExtentPanner {};

    /** @brief Modify an extent parameter given a distance.
     *
     * A right triangle if formed, with the adjacent edge being the distance,
     * and the opposite edge being determined from the extent. The angle formed
     * is then used to determine the new extent.
     *
     * - at distance=0, the extent is always 360
     * - at distance=1, the original extent is used
     * - at distance>1, the extent decreases
     * - in 0 < distance < 1, the extent changes more steeply around 0 for
     * smaller extents
     */
    double extentMod(double extent, double distance);

    /** @brief Calculate basis vectors that rotate (0, 1, 0)
       onto source_pos. */
    Eigen::Matrix3d calcBasis(Eigen::Vector3d position);

    /** @brief Polar to Cartesian in radians with no distance, in a given
     * basis.*/
    Eigen::Vector3d cartOnBasis(Eigen::Matrix3d basis, double azimuth,
                                double elevation);

    std::pair<double, double> azimuthElevationOnBasis(
        Eigen::Matrix3d basis, Eigen::RowVector3d position);

    /** @brief Weighting function for spread sources.
     *
     * The weighting function is one inside a region approximately determined by
     * a width x height rectangle in azimuth-elevation space, with
     * maximally-sized rounded corners; the shape of the corners is calculated
     * using the vector angle from their centres (always directly above or below
     * the source position) so as to avoid issues at the poles.
     *
     * The two straight edges of the rectangle are always parallel in Cartesian
     * space; this is achieved by following azimuth lines; for tall sources, the
     * whole coordinate system is rotated 90 degrees about the source position
     * to achieve this.
     *
     * Note that for sources where width == height, this degrades to a circular
     * region relative to the source position.
     *
     * To make the two ends meet, the width is adjusted such that a width of 180
     * degrees is mapped to width + height.
     *
     */
    class WeightingFunction {
     public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /** @brief Ctor
       *
       * @param position Centre of the extent.
       * @param width Width of the extent in degrees from one edge to the other.
       * @param height Height of the extent in degrees from one edge to the
       * other.
       */
      WeightingFunction(Eigen::Vector3d position, double width, double height);

      /** @brief Calculate weight for position */
      double operator()(Eigen::Vector3d position) const;

     private:
      const double _fadeWidth = 10.0;
      double _width;
      double _height;
      double _circleRadius;
      Eigen::Matrix3d _flippedBasis;
      double _circlePos;
      Eigen::Matrix<double, 3, 2> _circlePositions;
    };

    class SpreadingPanner {
     public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      SpreadingPanner(std::shared_ptr<PointSourcePanner> psp, int nRows);

      /** @brief Panning values for a given weighting function.
       *
       * @param weightFunc function from Cartesian position to weight in range
       * (0, 1)
       *
       * @return panning value for each speaker.
       */
      Eigen::VectorXd panningValuesForWeight(
          const WeightingFunction& weightFunc);

     private:
      /** @brief Generate points spread evenly on the sphere.
       *
       * Based on
       * http://web.archive.org/web/20150108040043/http://www.math.niu.edu/~rusin/known-math/95/equispace.elect
       *
       * @param nRows number of rows to place on sphere, e.g. 37 for 5 degree
       * spacing
       *
       * @returns cartesian array.
       */
      Eigen::MatrixXd _generatePanningPositionsEven();
      Eigen::MatrixXd _generatePanningPositionsResults();

      std::shared_ptr<PointSourcePanner> _psp;
      int _nRows;
      Eigen::MatrixXd _panningPositions;
      Eigen::MatrixXd _panningPositionsResults;
    };

    class PolarExtentPanner : public ExtentPanner {
     public:
      PolarExtentPanner(std::shared_ptr<PointSourcePanner> psp);

      /** @brief Calculate loudspeaker gains given position and extent
       * parameters.
       *
       * @param position  Cartesian source position
       * @param width block format width parameter
       * @param height block format height parameter
       * @param depth block format depth parameter
       *
       * @returns loudspeaker gains for each channel
       */
      Eigen::VectorXd handle(Eigen::Vector3d position, double width,
                             double height, double depth);

      /** @brief Calculate the speaker panning values for the position, width,
       * and height of a source; this just deals with the positioning and
       * spreading.
       */
      Eigen::VectorXd calcPvSpread(Eigen::Vector3d position, double width,
                                   double height);

     private:
      const int _nRows = 37;  // 5 degrees per row
      const double _fadeWidth = 10.0;  // degrees
      std::shared_ptr<PointSourcePanner> _psp;
      SpreadingPanner _spreadingPanner;
    };

    class CartesianExtentPanner : public ExtentPanner {};

  }  // namespace reference
}  // namespace ear
