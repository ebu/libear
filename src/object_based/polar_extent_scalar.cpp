#include <boost/make_unique.hpp>
#include <cmath>
#include "polar_extent_core.hpp"

namespace ear {

  struct ExtentPosition {
    extent_float_t x;
    extent_float_t y;
    extent_float_t z;

    extent_float_t dot(const extent_float_t *vec) const {
      return x * vec[0] + y * vec[1] + z * vec[2];
    }

    ExtentPosition transform(const extent_float_t *m) const {
      ExtentPosition res;
      res.x = dot(m);
      res.y = dot(m + 3);
      res.z = dot(m + 6);
      return res;
    }
  };

  /// fallback scalar implementation of PolarExtentCore; see PolarExtent
  class PolarExtentCoreScalar : public PolarExtentCore {
   public:
    virtual size_t alignment() const override {
      // required for posix_memalign
      return sizeof(void *);
    }
    virtual size_t batch_size() const override { return 1; }

    extent_float_t weight_from_cos(PolarExtentCoreContext &ctx,
                                   extent_float_t cos_angle) const {
      if (cos_angle >= ctx.cos_start_angle) return 1.0;
      if (cos_angle <= ctx.cos_end_angle) return 0.0;

      return ctx.m * std::acos(cos_angle) + ctx.c;
    }

    extent_float_t weight_from_sin(PolarExtentCoreContext &ctx,
                                   extent_float_t sin_angle) const {
      if (sin_angle <= ctx.sin_start_angle) return 1.0;
      if (sin_angle >= ctx.sin_end_angle) return 0.0;

      return ctx.m * std::asin(sin_angle) + ctx.c;
    }

    extent_float_t weight_circle(PolarExtentCoreContext &ctx,
                                 const ExtentPosition &position) const {
      // simplified dot product assuming that circle_centre is {0, 1, 0}
      auto dot = position.dot(ctx.flippedBasis + 3);
      return weight_from_cos(ctx, dot);
    }

    extent_float_t weight_stadium(PolarExtentCoreContext &ctx,
                                  const ExtentPosition &position) const {
      ExtentPosition position_t = position.transform(ctx.flippedBasis);

      ExtentPosition position_t_right = position_t;
      position_t_right.x = std::abs(position_t.x);

      auto circle_test_dot = position_t_right.x * ctx.circle_test[0] +
                             position_t_right.y * ctx.circle_test[1];
      bool in_straight_line_part = circle_test_dot >= 0.0;

      if (in_straight_line_part)
        return weight_from_sin(ctx, std::abs(position_t.z));
      else {
        auto circle_dot = position_t_right.x * ctx.right_circle_centre[0] +
                          position_t_right.y * ctx.right_circle_centre[1];
        return weight_from_cos(ctx, circle_dot);
      }
    }

    void run(PolarExtentCoreContext &ctx) const override {
      for (size_t speaker_idx = 0; speaker_idx < ctx.num_speakers;
           speaker_idx++)
        ctx.results[speaker_idx] = 0.0;

      for (size_t i = 0; i < ctx.num_points; i++) {
        ExtentPosition pos;
        pos.x = ctx.xs[i];
        pos.y = ctx.ys[i];
        pos.z = ctx.zs[i];

        extent_float_t weight = ctx.is_circular ? weight_circle(ctx, pos)
                                                : weight_stadium(ctx, pos);

        // add to results, using summed_panning_results for both as this is in
        // a more efficient order
        if (weight == 1.0) {
          for (size_t speaker_idx = 0; speaker_idx < ctx.num_speakers;
               speaker_idx++) {
            extent_float_t gain =
                ctx.summed_panning_results[i * ctx.num_speakers + speaker_idx];
            ctx.results[speaker_idx] += gain;
          }
        } else if (weight != 0) {
          for (size_t speaker_idx = 0; speaker_idx < ctx.num_speakers;
               speaker_idx++) {
            extent_float_t gain =
                ctx.summed_panning_results[i * ctx.num_speakers + speaker_idx];
            ctx.results[speaker_idx] += weight * gain;
          }
        }
      }
    }
  };

  std::unique_ptr<PolarExtentCore> get_polar_extent_core_scalar() {
    return boost::make_unique<PolarExtentCoreScalar>();
  }
}  // namespace ear
