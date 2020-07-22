#pragma once
#include <cmath>
#include "polar_extent_core.hpp"
#include "xsimd/xsimd.hpp"

namespace ear {

  template <typename T>
  struct PositionBatch {
    T x;
    T y;
    T z;

    T dot(const extent_float_t *vec) const {
      return xsimd::fma(x, T{vec[0]}, xsimd::fma(y, T{vec[1]}, z * T{vec[2]}));
    }

    PositionBatch<T> transform(const extent_float_t *m) const {
      PositionBatch<T> res;
      res.x = dot(m);
      res.y = dot(m + 3);
      res.z = dot(m + 6);
      return res;
    }
  };

  /// SIMD implementation of PolarExtentCore for a given architecture; see
  /// PolarExtent
  ///
  /// these are to be instantiated in polar_extent_simd_instance.cpp
  template <typename arch>
  class PolarExtentCoreSimd : public PolarExtentCore {
   public:
    using batch = xsimd::batch<extent_float_t, arch>;

    virtual size_t alignment() const override { return arch::alignment(); }
    virtual size_t batch_size() const override { return batch::size; }

    batch weight_from_cos(PolarExtentCoreContext &ctx, batch cos_angle) const {
      auto start = cos_angle >= ctx.cos_start_angle;
      auto end = cos_angle <= ctx.cos_end_angle;
      if (xsimd::all(start)) return batch{1.0};
      if (xsimd::all(end)) return batch{0.0};

      auto ramp = ctx.m * xsimd::acos(cos_angle) + ctx.c;
      return xsimd::select(start, batch{1.0},
                           xsimd::select(end, batch{0.0}, ramp));
    }

    batch weight_from_sin(PolarExtentCoreContext &ctx, batch sin_angle) const {
      auto start = sin_angle <= ctx.sin_start_angle;
      auto end = sin_angle >= ctx.sin_end_angle;
      if (xsimd::all(start)) return batch{1.0};
      if (xsimd::all(end)) return batch{0.0};

      auto ramp = ctx.m * xsimd::asin(sin_angle) + ctx.c;
      return xsimd::select(start, batch{1.0},
                           xsimd::select(end, batch{0.0}, ramp));
    }

    batch weight_circle(PolarExtentCoreContext &ctx,
                        const PositionBatch<batch> &position) const {
      // simplified dot product assuming that circle_centre is {0, 1, 0}
      auto dot = position.dot(ctx.flippedBasis + 3);
      return weight_from_cos(ctx, dot);
    }

    batch weight_stadium(PolarExtentCoreContext &ctx,
                         const PositionBatch<batch> &position) const {
      PositionBatch<batch> position_t = position.transform(ctx.flippedBasis);

      PositionBatch<batch> position_t_right = position_t;
      position_t_right.x = xsimd::abs(position_t.x);

      auto circle_test_dot =
          xsimd::fma(position_t_right.x, batch{ctx.circle_test[0]},
                     position_t_right.y * batch{ctx.circle_test[1]});
      auto in_straight_line_part = circle_test_dot >= batch{0.0};

      // for the straight line part
      auto straight_val = weight_from_sin(ctx, xsimd::abs(position_t.z));

      // for the circle part
      auto circle_dot =
          xsimd::fma(position_t_right.x, batch{ctx.right_circle_centre[0]},
                     position_t_right.y * batch{ctx.right_circle_centre[1]});
      auto circle_val = weight_from_cos(ctx, circle_dot);

      if (xsimd::all(in_straight_line_part))
        return straight_val;
      else if (xsimd::none(in_straight_line_part))
        return circle_val;
      else
        return xsimd::select(in_straight_line_part, straight_val, circle_val);
    }

    void run(PolarExtentCoreContext &ctx) const override {
      for (size_t speaker_idx = 0; speaker_idx < ctx.num_speakers;
           speaker_idx++)
        ctx.results[speaker_idx] = 0.0;

      for (size_t i = 0; i < ctx.num_points; i += batch::size) {
        PositionBatch<batch> pos;
        pos.x = batch::load_aligned(ctx.xs + i);
        pos.y = batch::load_aligned(ctx.ys + i);
        pos.z = batch::load_aligned(ctx.zs + i);

        batch weight = ctx.is_circular ? weight_circle(ctx, pos)
                                       : weight_stadium(ctx, pos);

        bool all_zeros = xsimd::all(weight == 0.0);
        bool all_ones = xsimd::all(weight == 1.0);
        if (all_ones) {
          // add pre-computed sum for this batch
          size_t batch_i = i / batch::size;
          for (size_t speaker_idx = 0; speaker_idx < ctx.num_speakers;
               speaker_idx += batch::size) {
            batch gain =
                batch::load_aligned(ctx.summed_panning_results +
                                    (batch_i * ctx.num_speakers) + speaker_idx);

            extent_float_t *result_ptr = ctx.results + speaker_idx;
            xsimd::store_aligned(result_ptr,
                                 batch::load_aligned(result_ptr) + gain);
          }
        } else if (!all_zeros) {
          for (size_t speaker_idx = 0; speaker_idx < ctx.real_num_speakers;
               speaker_idx++) {
            batch gain = batch::load_aligned(ctx.panning_results +
                                             speaker_idx * ctx.num_points + i);
            ctx.results[speaker_idx] += xsimd::reduce_add(gain * weight);
          }
        }
      }
    }
  };

}  // namespace ear
