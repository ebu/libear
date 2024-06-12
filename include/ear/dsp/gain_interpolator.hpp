#pragma once
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "../helpers/assert.hpp"

namespace ear {
  namespace dsp {
    /// \file
    /// Utilities for applying interpolated gain vectors to audio samples; see
    /// ear::dsp::GainInterpolator for details.

    /// Type used to index into sample buffers.
    using SampleIndex = long int;

    /// Gain interpolator, templated over an interpolation type which defines
    /// the type of interpolation (linear, cosine etc.), the type of the values
    /// to interpolate between (floats, vectors, matrices), and therefore
    /// restrictions on the input and output channel sizes.
    ///
    /// An interpolation curve is defined by the points in #interp_points. Each
    /// of these is a pair of the sample index and the gain values at that time.
    /// These must be sorted in time order. Duplicate times can be used to
    /// specify steps.
    ///
    /// See LinearInterpSingle, LinearInterpVector and LinearInterpMatrix for
    /// possible interpolation types. See InterpType for the interface that
    /// interpolation types define.
    //
    // Internally, blocks of samples between two interpolation points are
    // referred to by the 'block index', which is the index of the end of the
    // block in interp_points, except for interp_points.size(), which doesn't
    // have an end. An example with 3 interpolation points:
    //
    // interp_points:   0   1   2
    // block_idx:     0 | 1 | 2 | 3
    template <typename InterpType>
    class GainInterpolator {
     public:
      std::vector<std::pair<SampleIndex, typename InterpType::Point>>
          interp_points;

      /// Process n samples.
      /// \param block_start the index of the first sample relative to the
      ///     sample indices in #interp_points
      /// \param nsamples number of samples in \p in and \p out
      /// \param in input samples with a number of channels compatible with the
      ///     interpolation type and points used
      /// \param out output samples with a number of channels compatible with
      ///     the interpolation type and points used
      void process(SampleIndex block_start, size_t nsamples,
                   const float *const *in, float *const *out) {
        SampleIndex block_end = block_start + (SampleIndex)nsamples;
        SampleIndex this_block_start = block_start;

        while (this_block_start < block_end) {
          size_t block_idx = find_block(this_block_start);

          SampleIndex this_block_end =
              block_idx == interp_points.size()
                  ? block_end
                  : std::min(interp_points[block_idx].first, block_end);
          ear_assert(this_block_start < this_block_end,
                     "found block ends before processed block starts");

          if (block_idx == 0 || block_idx == interp_points.size() ||
              InterpType::constant_interp(interp_points[block_idx - 1].second,
                                          interp_points[block_idx].second)) {
            size_t point_with_value =
                block_idx == interp_points.size() ? block_idx - 1 : block_idx;
            InterpType::apply_constant(in, out, this_block_start - block_start,
                                       this_block_end - block_start,
                                       interp_points[point_with_value].second);
          } else {
            InterpType::apply_interp(in, out, this_block_start - block_start,
                                     this_block_end - block_start, block_start,
                                     interp_points[block_idx - 1].first,
                                     interp_points[block_idx].first,
                                     interp_points[block_idx - 1].second,
                                     interp_points[block_idx].second);
          }

          this_block_start = this_block_end;
        }
      }

     private:
      // find whether a sample is before, after or inside a block:
      //  0: sample is inside the block
      // -1: sample is before the start of the block
      //  1: sample is after the end of the block
      int block_cmp(size_t block_idx, SampleIndex sample_idx) {
        if (block_idx > 0) {
          SampleIndex block_start = interp_points[block_idx - 1].first;
          if (sample_idx < block_start) return -1;
        }

        if (block_idx < interp_points.size()) {
          SampleIndex block_end = interp_points[block_idx].first;
          if (sample_idx >= block_end) return 1;
        }

        return 0;
      }

      // find the block index for a sample; the last block returned is cached,
      // so in common cases we don't have to look at many blocks
      size_t last_block = 0;
      size_t find_block(SampleIndex sample_idx) {
        // last_block could be outside the allowed range if interp_points
        // changed since the last call
        if (last_block > interp_points.size()) last_block = 0;

        // move in the direction given by block_cmp until we find the right
        // block (cmp == 0); this is slightly complicated by checking that we
        // are making progress.
        int cmp = block_cmp(last_block, sample_idx);
        int first_cmp = cmp;
        while (cmp != 0) {
          last_block += cmp;
          if (cmp != first_cmp)
            ear_throw(invalid_argument("interpolation points are not sorted"));
          cmp = block_cmp(last_block, sample_idx);
        }

        return last_block;
      }
    };

    /// Base type for interpolation types.
    template <typename PointT>
    struct InterpType {
      /// Type of points on the interpolation curve, for example float, vector,
      /// matrix.
      using Point = PointT;

      /// Are the two points the same (and therefore constant/no interpolation
      /// should be used between them)?
      static bool constant_interp(const Point &a, const Point &b) {
        return a == b;
      }

      /// Apply interpolated gains to \p in, writing to \p out.
      ///
      /// For example, if an interpolation curve goes from x to y between
      /// sample 5 and 15, these calls would occur for the first and second
      /// 10-sample blocks:
      ///
      /// \code
      /// apply_interp(in_a, out_a, 5, 10, 0, 5, 15, x, y);
      /// apply_interp(in_b, out_b, 0, 5, 10, 5, 15, x, y);
      /// \endcode
      ///
      /// \param in input samples
      /// \param out output samples
      /// \param range_start offset in \p in and \p out to start processing
      /// \param range_end offset in \p in and \p out to end processing
      /// \param block_start: start sample index of this block, i.e. in[0][0]
      /// \param start: start sample index of interpolation curve
      /// \param end: end sample index of interpolation curve
      /// \param start_point: gain values at \p start
      /// \param end_point: gain values at \p end
      static void apply_interp(const float *const *in, float *const *out,
                               SampleIndex range_start, SampleIndex range_end,
                               SampleIndex block_start, SampleIndex start,
                               SampleIndex end, const Point &start_point,
                               const Point &end_point);

      /// Apply constnt gain gains to \p in, writing to \p out.
      ///
      /// \param in input samples
      /// \param out output samples
      /// \param range_start offset in \p in and \p out to start processing
      /// \param range_end offset in \p in and \p out to end processing
      /// \param point: gain values to apply
      static void apply_constant(const float *const *in, float *const *out,
                                 SampleIndex range_start, SampleIndex range_end,
                                 const Point &point);
    };

    // interpolation implementations

    /// Linear interpolation of a single channel for use in GainInterpolator.
    struct LinearInterpSingle : public InterpType<float> {
      static void apply_interp(const float *const *in, float *const *out,
                               SampleIndex range_start, SampleIndex range_end,
                               SampleIndex block_start, SampleIndex start,
                               SampleIndex end, const Point &start_point,
                               const Point &end_point) {
        float scale = 1.0f / (end - start);
        for (SampleIndex i = range_start; i < range_end; i++) {
          float p = (float)((block_start + i) - start) * scale;

          float gain = (1.0f - p) * start_point + p * end_point;

          out[0][i] = in[0][i] * gain;
        }
      }

      static void apply_constant(const float *const *in, float *const *out,
                                 SampleIndex range_start, SampleIndex range_end,
                                 const Point &point) {
        for (SampleIndex i = range_start; i < range_end; i++) {
          out[0][i] = in[0][i] * point;
        }
      }
    };

    /// Linear interpolation with one channel in and multiple out for use in
    /// GainInterpolator.
    struct LinearInterpVector : public InterpType<std::vector<float>> {
      static void apply_interp(const float *const *in, float *const *out,
                               SampleIndex range_start, SampleIndex range_end,
                               SampleIndex block_start, SampleIndex start,
                               SampleIndex end, const Point &start_point,
                               const Point &end_point) {
        float scale = 1.0f / (end - start);

        for (size_t channel = 0; channel < start_point.size(); channel++) {
          float s = start_point[channel];
          float e = end_point[channel];

          for (SampleIndex i = range_start; i < range_end; i++) {
            float p = (float)((block_start + i) - start) * scale;
            float gain = (1.0f - p) * s + p * e;
            out[channel][i] = in[0][i] * gain;
          }
        }
      }

      static void apply_constant(const float *const *in, float *const *out,
                                 SampleIndex range_start, SampleIndex range_end,
                                 const Point &point) {
        for (size_t channel = 0; channel < point.size(); channel++) {
          for (SampleIndex i = range_start; i < range_end; i++) {
            out[channel][i] = in[0][i] * point[channel];
          }
        }
      }
    };

    /// Linear interpolation with multiple input and output channels, with a
    /// matrix of coefficients for use in GainInterpolator.
    ///
    /// Points contain one vector of per-output gains per input channel.
    struct LinearInterpMatrix
        : public InterpType<std::vector<std::vector<float>>> {
      static void apply_interp(const float *const *in, float *const *out,
                               SampleIndex range_start, SampleIndex range_end,
                               SampleIndex block_start, SampleIndex start,
                               SampleIndex end, const Point &start_point,
                               const Point &end_point) {
        float scale = 1.0f / (end - start);
        for (size_t out_channel = 0;
             out_channel < (start_point.size() ? start_point[0].size() : 0);
             out_channel++) {
          for (SampleIndex i = range_start; i < range_end; i++) {
            out[out_channel][i] = 0.0;
          }
        }

        for (size_t in_channel = 0; in_channel < start_point.size();
             in_channel++) {
          for (size_t out_channel = 0;
               out_channel < start_point[in_channel].size(); out_channel++) {
            float s = start_point[in_channel][out_channel];
            float e = end_point[in_channel][out_channel];

            for (SampleIndex i = range_start; i < range_end; i++) {
              float p = (float)((block_start + i) - start) * scale;
              float gain = (1.0f - p) * s + p * e;
              out[out_channel][i] += in[in_channel][i] * gain;
            }
          }
        }
      }

      static void apply_constant(const float *const *in, float *const *out,
                                 SampleIndex range_start, SampleIndex range_end,
                                 const Point &point) {
        for (size_t out_channel = 0;
             out_channel < (point.size() ? point[0].size() : 0);
             out_channel++) {
          for (SampleIndex i = range_start; i < range_end; i++) {
            out[out_channel][i] = 0.0;
          }
        }
        for (size_t in_channel = 0; in_channel < point.size(); in_channel++) {
          for (size_t out_channel = 0; out_channel < point[in_channel].size();
               out_channel++) {
            for (SampleIndex i = range_start; i < range_end; i++) {
              out[out_channel][i] +=
                  in[in_channel][i] * point[in_channel][out_channel];
            }
          }
        }
      }
    };
  }  // namespace dsp
}  // namespace ear
