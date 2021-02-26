#pragma once
#include <Eigen/Core>
#include <memory>
#include "../common/point_source_panner.hpp"

namespace ear {

  class ExtentPanner {};

  /** @brief Modify an extent parameter given a distance.
   *
   * A right triangle if formed, with the adjacent edge being the distance, and
   * the opposite edge being determined from the extent. The angle formed is
   * then used to determine the new extent.
   *
   * - at distance=0, the extent is always 360
   * - at distance=1, the original extent is used
   * - at distance>1, the extent decreases
   * - in 0 < distance < 1, the extent changes more steeply around 0 for smaller
   *   extents
   */
  double extentMod(double extent, double distance);

  /// calculates the extent weight for a given angle, in terms of the sin or
  /// cos of that angle
  class AngleToWeight {
   public:
    AngleToWeight();
    /// angles < start_angle yield 1.0
    /// angles > end_angle yield 0.0
    /// other angles are interpolated
    AngleToWeight(double start_angle, double end_angle);

    double from_cos(double cos_angle) const;
    double from_sin(double sin_angle) const;

    // is an angle inside or outside the extent shape?
    // these are here to allow them to be inlined in HTM
    bool inside_from_cos(double cos_angle) const {
      return cos_angle >= cos_start_angle;
    }
    bool outside_from_cos(double cos_angle) const {
      return cos_angle <= cos_end_angle;
    }
    bool inside_from_sin(double sin_angle) const {
      return sin_angle <= sin_start_angle;
    }
    bool outside_from_sin(double sin_angle) const {
      return sin_angle >= sin_end_angle;
    }

   private:
    double cos_start_angle, cos_end_angle;
    double sin_start_angle, sin_end_angle;
    double m, c;
  };

  /** @brief Calculate basis vectors that rotate (0, 1, 0)
     onto source_pos. */
  Eigen::Matrix3d calcBasis(Eigen::Vector3d position);

  /** @brief Weighting function for spread sources.
   *
   * The weighting function is one inside a region approximately determined by a
   * width x height rectangle in azimuth-elevation space, with maximally-sized
   * rounded corners; the shape of the corners is calculated using the vector
   * angle from their centres (always directly above or below the source
   * position) so as to avoid issues at the poles.
   *
   * The two straight edges of the rectangle are always parallel in Cartesian
   * space; this is achieved by following azimuth lines; for tall sources, the
   * whole coordinate system is rotated 90 degrees about the source position to
   * achieve this.
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
     * @param height Height of the extent in degrees from one edge to the other.
     */
    WeightingFunction(Eigen::Vector3d position, double width, double height);

    /** @brief Calculate weight for position */
    double operator()(
        const Eigen::Ref<const Eigen::RowVector3d> &position) const;

   private:
    // pointer to the actual weighting function, one of the static weight_*
    // below
    double (*weight_cb)(const WeightingFunction &self,
                        const Eigen::Ref<const Eigen::RowVector3d> &position);

    static double weight_circle(
        const WeightingFunction &self,
        const Eigen::Ref<const Eigen::RowVector3d> &position);
    static double weight_stadium(
        const WeightingFunction &self,
        const Eigen::Ref<const Eigen::RowVector3d> &position);

    const double _fadeWidth = 10.0;
    double _width;
    double _height;
    // angle of the circle centres from the source position
    double circlePos() const { return _width - _height; }
    double circleRadius() const { return _height; }
    Eigen::Matrix3d _flippedBasis;

    // xy position of the right circle centre, used to measure the distance via
    // dot product
    Eigen::Vector2d right_circle_centre;
    // vector in xy to test if a point is between the circles
    Eigen::Vector2d circle_test;

    AngleToWeight angle_to_weight;

    // is the extent shape circular? this simplifies some logic
    bool is_circular;

    friend class WeightingFunctionEdges;
  };

  class SpreadingPannerBase {
   public:
    /** @brief Panning values for a given weighting function.
     *
     * @param weightFunc function from Cartesian position to weight in range
     * (0, 1)
     *
     * @return panning value for each speaker.
     */
    virtual Eigen::VectorXd panningValuesForWeight(
        const WeightingFunction &weightFunc) = 0;

    virtual ~SpreadingPannerBase() {}

   protected:
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
    static Eigen::MatrixXd _generatePanningPositionsEven(int nRows);
    static Eigen::MatrixXd _generatePanningPositionsResults(
        std::shared_ptr<PointSourcePanner> psp,
        const Eigen::Ref<const Eigen::MatrixXd> &positions);
  };

  class SpreadingPanner : public SpreadingPannerBase {
   public:
    SpreadingPanner(std::shared_ptr<PointSourcePanner> psp, int nRows);
    Eigen::VectorXd panningValuesForWeight(
        const WeightingFunction &weightFunc) override;

   private:
    std::shared_ptr<PointSourcePanner> _psp;
    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> _panningPositions;
    Eigen::MatrixXd _panningPositionsResults;
  };

  class PolarExtentPanner : public ExtentPanner {
   public:
    /// construct with the default spreading panner
    PolarExtentPanner(std::shared_ptr<PointSourcePanner> psp);

    /// construct with the given spreading panner, mostly useful for testing
    PolarExtentPanner(std::shared_ptr<PointSourcePanner> psp,
                      std::unique_ptr<SpreadingPannerBase> spreadingPanner);

    /** @brief Calculate loudspeaker gains given position and extent parameters.
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

    /** @brief Calculate the speaker panning values for the position, width, and
     * height of a source; this just deals with the positioning and spreading.
     */
    Eigen::VectorXd calcPvSpread(Eigen::Vector3d position, double width,
                                 double height);

    static const int nRowsDefault;

   private:
    const double _fadeWidth = 10.0;  // degrees
    std::shared_ptr<PointSourcePanner> _psp;
    std::unique_ptr<SpreadingPannerBase> _spreadingPanner;
  };

  class CartesianExtentPanner : public ExtentPanner {};

}  // namespace ear
