#include "polar_extent.hpp"
#include <boost/math/constants/constants.hpp>
#include <xsimd/memory/xsimd_aligned_allocator.hpp>
#include "../common/geom.hpp"
#include "../common/helpers/eigen_helpers.hpp"

const double PI = boost::math::constants::pi<double>();

namespace ear {

  namespace {
    constexpr double fadeWidth = 10.0;
    constexpr int nRows = 37;  // 5 degrees per row

    Eigen::MatrixXd generatePanningPositionsEven(int nRows) {
      Eigen::VectorXd elevations =
          Eigen::VectorXd::LinSpaced(nRows, -90.0, 90.0);
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

    Eigen::MatrixXd generatePanningPositionsResults(
        const std::shared_ptr<PointSourcePanner> &psp,
        const Eigen::Ref<const Eigen::MatrixXd> &positions) {
      Eigen::MatrixXd results(psp->numberOfOutputChannels(), positions.rows());
      for (int i = 0; i < positions.rows(); ++i) {
        results.col(i) = psp->handle(positions.row(i)).get();
      }
      return results;
    }

    size_t round_up(size_t n, size_t multiple) {
      return ((n + multiple - 1) / multiple) * multiple;
    }

    /** @brief Normalise position or return {0,1,0}.
     *
     * @param position  Position to normalise.
     *
     * @returns normalised position
     */
    static Eigen::Vector3d safeNormPosition(Eigen::Vector3d position) {
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
  }  // namespace

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

  PolarExtentCore::~PolarExtentCore() {}

  PolarExtent::PolarExtent(const std::shared_ptr<PointSourcePanner> &psp_)
      : PolarExtent(psp_, get_polar_extent_core()) {}

  PolarExtent::PolarExtent(const std::shared_ptr<PointSourcePanner> &psp_,
                           std::unique_ptr<PolarExtentCore> core_impl_)
      : core_impl(std::move(core_impl_)),
        psp(psp_),
        pvsMin(psp->numberOfOutputChannels()),
        pvsMax(psp->numberOfOutputChannels()) {
    auto panning_positions = generatePanningPositionsEven(nRows);
    auto panning_results =
        generatePanningPositionsResults(psp, panning_positions);

    size_t num_speakers = panning_results.rows();
    size_t num_points = panning_results.cols();
    size_t batch_size = core_impl->batch_size();
    ctx.real_num_speakers = num_speakers;
    ctx.num_speakers = round_up(num_speakers, batch_size);
    ctx.num_points = round_up(num_points, batch_size);

    // use regular malloc and free for memory management. reasoning:
    // - the parts compiled with different architectures should not use any
    //   in-line functions which could get merged; therefore using any kind of
    //   smart pointer in the interface is a bad plan
    // - we could use a smart pointer and extract a float pointer, but that
    //   doesn't make it much cleaner, would still break copying, and would
    //   ultimately be more code
    // - using smart pointers would be awkward anyway, as the alignment is
    //   known at run-time
    auto malloc_float = [&](size_t n) {
      auto mem = (extent_float_t *)xsimd::aligned_malloc(
          sizeof(extent_float_t) * n, core_impl->alignment());
      for (size_t i = 0; i < n; i++) mem[i] = 0.0;
      return mem;
    };

    ctx.xs = malloc_float(ctx.num_points);
    ctx.ys = malloc_float(ctx.num_points);
    ctx.zs = malloc_float(ctx.num_points);

    ctx.panning_results = malloc_float(ctx.num_points * ctx.num_speakers);
    size_t num_batches = ctx.num_points / batch_size;
    ctx.summed_panning_results = malloc_float(num_batches * ctx.num_speakers);

    ctx.results = malloc_float(ctx.num_speakers);

    for (size_t p = 0; p < num_points; p++) {
      ctx.xs[p] = static_cast<extent_float_t>(panning_positions(p, 0));
      ctx.ys[p] = static_cast<extent_float_t>(panning_positions(p, 1));
      ctx.zs[p] = static_cast<extent_float_t>(panning_positions(p, 2));

      for (size_t s = 0; s < num_speakers; s++) {
        ctx.panning_results[s * ctx.num_points + p] =
            static_cast<extent_float_t>(panning_results(s, p));
        size_t batch = p / batch_size;
        ctx.summed_panning_results[batch * ctx.num_speakers + s] +=
            static_cast<extent_float_t>(panning_results(s, p));
      }
    }

    for (size_t p = num_points; p < ctx.num_points; p++) {
      // the results of these will be ignored; make them consistent with the
      // rest of the batch
      ctx.xs[p] =
          static_cast<extent_float_t>(panning_positions(num_points - 1, 0));
      ctx.ys[p] =
          static_cast<extent_float_t>(panning_positions(num_points - 1, 1));
      ctx.zs[p] =
          static_cast<extent_float_t>(panning_positions(num_points - 1, 2));
    }
  }

  PolarExtent::~PolarExtent() {
    xsimd::aligned_free(ctx.xs);
    xsimd::aligned_free(ctx.ys);
    xsimd::aligned_free(ctx.zs);
    xsimd::aligned_free(ctx.panning_results);
    xsimd::aligned_free(ctx.summed_panning_results);
    xsimd::aligned_free(ctx.results);
  }

  void PolarExtent::setup_angle_to_weight(double start_angle,
                                          double end_angle) const {
    ear_assert(start_angle >= 0, "start angle should be +ve");
    ear_assert(end_angle >= 0, "end angle should be +ve");
    ear_assert(end_angle > start_angle, "end angle should be > start angle");

    ctx.cos_start_angle = static_cast<extent_float_t>(
        start_angle < PI ? std::cos(start_angle) : -1.0);
    ctx.cos_end_angle = static_cast<extent_float_t>(
        end_angle < PI ? std::cos(end_angle) : -(1.0 + 1e-6));
    // sin is only used for less than PI/2
    ctx.sin_start_angle = static_cast<extent_float_t>(
        start_angle < PI / 2 ? std::sin(start_angle) : 1.0);
    ctx.sin_end_angle = static_cast<extent_float_t>(
        end_angle < PI / 2 ? std::sin(end_angle) : 1.0 + 1e-6);
    // between start and end angle, we want:
    // out = (angle - end_angle) / (start_angle - end_angle)
    // therefore the slope is:
    ctx.m = static_cast<extent_float_t>(1.0 / (start_angle - end_angle));
    // out = m * (angle - end_angle)
    // out = m * angle - m * end_angle
    // intercept is:
    ctx.c = static_cast<extent_float_t>(-ctx.m * end_angle);
  }

  void PolarExtent::setup_weighting_function(Eigen::Vector3d position,
                                             double width,
                                             double height) const {
    width = radians(width) / 2;
    height = radians(height) / 2;

    // basis vectors to rotate the vsource positions towards position
    Eigen::Matrix3d m = calcBasis(position);

    // Flip the width and the height such that it is always wider than it is
    // high from here in.
    if (height > width) {
      std::swap(height, width);
      // rotate rather than flipping x and y to preserve triangle winding
      Eigen::Matrix3d flip;
      flip << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;
      m = flip * m;
    }

    for (size_t i = 0; i < 3; i++)
      for (size_t j = 0; j < 3; j++)
        ctx.flippedBasis[i * 3 + j] = static_cast<extent_float_t>(m(i, j));

    // modify the width to make it meet at the back.
    double widthFull = PI + height;
    // interpolate to this from a width of pi/2 to pi
    double widthMod = interp(width, Eigen::Vector3d{0.0, PI / 2.0, PI},
                             Eigen::Vector3d{0.0, PI / 2.0, widthFull});
    // apply this fully for a height of less than pi/4; tail off until pi/2
    width = interp(height, Eigen::Vector4d{0, PI / 4.0, PI / 2.0, PI},
                   Eigen::Vector4d{widthMod, widthMod, width, width});

    ctx.is_circular = (width - height) < 1e-6;

    double circlePos = width - height;

    ctx.right_circle_centre[0] =
        static_cast<extent_float_t>(std::sin(circlePos));
    ctx.right_circle_centre[1] =
        static_cast<extent_float_t>(std::cos(circlePos));
    ctx.circle_test[0] = static_cast<extent_float_t>(-std::cos(circlePos));
    ctx.circle_test[1] = static_cast<extent_float_t>(std::sin(circlePos));

    setup_angle_to_weight(height, height + radians(fadeWidth));
  }

  void PolarExtent::calc_pv_spread(Eigen::Vector3d position, double width,
                                   double height,
                                   Eigen::Ref<Eigen::VectorXd> out) const {
    // When calculating the spread panning values the width and height are
    // set to at least fade_width. For sizes where any of the dimensions is
    // less than this, interpolate linearly between the point and spread
    // panning values.
    double ammount_spread =
        interp(std::max(width, height), Eigen::Vector2d(0.0, fadeWidth),
               Eigen::Vector2d(0.0, 1.0));
    double ammountPoint = 1.0 - ammount_spread;
    out.setZero();
    if (ammountPoint > 1e-10) {
      out.array() +=
          ammountPoint * psp->handle(position).get().array().square();
    }
    if (ammount_spread > 1e-10) {
      // minimum width and height as above
      width = std::max(width, fadeWidth / 2.0);
      height = std::max(height, fadeWidth / 2.0);

      setup_weighting_function(position, width, height);
      core_impl->run(ctx);

      auto results_map = Eigen::Map<Eigen::Matrix<extent_float_t, 1, -1>>(
          ctx.results, psp->numberOfOutputChannels());
      results_map *= 1.0 / results_map.norm();

      out.array() +=
          ammount_spread * results_map.array().square().cast<double>();
    }
    out.array() = out.array().sqrt();
  }

  void PolarExtent::handle(Eigen::Vector3d position, double width,
                           double height, double depth,
                           Eigen::Ref<Eigen::VectorXd> out) const {
    double distance = position.norm();

    if (depth != 0.0) {
      double distanceMin = distance - depth / 2.0;
      double distanceMax = distance + depth / 2.0;
      distanceMin = (distanceMin < 0) ? 0.0 : distanceMin;
      distanceMax = (distanceMax < 0) ? 0.0 : distanceMax;
      calc_pv_spread(position, extentMod(width, distanceMin),
                     extentMod(height, distanceMin), pvsMin);
      calc_pv_spread(position, extentMod(width, distanceMax),
                     extentMod(height, distanceMax), pvsMax);
      out = ((pvsMin.array().square() + pvsMax.array().square()) / 2.0)
                .sqrt()
                .matrix();
    } else {
      calc_pv_spread(position, extentMod(width, distance),
                     extentMod(height, distance), out);
    }
  }
}  // namespace ear
