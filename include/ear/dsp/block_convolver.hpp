#pragma once

#include <complex>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>
#include "../fft.hpp"
#include "ear/export.h"

namespace ear {
  namespace dsp {
    namespace block_convolver_impl {
      class Context;
      class Filter;
      class BlockConvolver;
    }  // namespace block_convolver_impl

    namespace block_convolver {
      /// Type for real data (float).
      using real_t = float;
      /// Type for complex data.
      using complex_t = std::complex<real_t>;

      /** Static data required to perform convolution of a particular block
       * size; may be shared between any number of BlockConvolver and Filter
       * instances. */
      class EAR_EXPORT Context {
       public:
        /** Create a Context with a given block size.
         * @param block_size Block size in samples.
         * @param fft_impl FFT implementation to use.
         */
        Context(size_t block_size, FFTImpl<real_t> &fft_impl);

       private:
        std::shared_ptr<block_convolver_impl::Context> impl;
        friend class Filter;
        friend class BlockConvolver;
      };

      /** A filter response which may be shared between many BlockConvolver
       * instances.
       *
       * This stores the pre-transformed filter blocks. */
      class EAR_EXPORT Filter {
       public:
        Filter(const Context &ctx, size_t n, const real_t *filter);

        /** The number of blocks in the filter. */
        size_t num_blocks() const;

       private:
        std::shared_ptr<block_convolver_impl::Filter> impl;
        friend class BlockConvolver;
      };

      /** BlockConvolver implements partitioned overlap-add convolution with a
       * fixed block size, with efficient fading between filters.
       */
      class EAR_EXPORT BlockConvolver {
       public:
        /** Create a BlockConvolver given the block size and number of blocks.
         * @param ctx Context required for transformations.
         * @param num_blocks Maximum number of blocks of any filter used.
         */
        BlockConvolver(const Context &ctx, size_t num_blocks);

        /** Create a BlockConvolver given the block size and number of blocks.
         *  If filter == nullptr, num_blocks must be specified.
         * @param ctx Context required for transformations.
         * @param filter Initial filter to be used, or nullptr for no filter.
         * @param num_blocks Maximum number of blocks of any filter used; using
         * 0 will take the number of blocks from the passed filter.
         */
        BlockConvolver(const Context &ctx, const Filter &filter,
                       size_t num_blocks = 0);

        ~BlockConvolver();

        /** Pass a block of audio through the filter.
         * @param in Input samples of length block_size
         * @param out Output samples of length block_size
         */
        void process(const float *in, float *out);

        /** Crossfade to a new filter during the next block.
         *
         * This is equivalent to:
         * - Creating a new convolver.
         * - Passing the next block of samples through the old and new
         * convolvers, with the input to the old faded down across the block,
         * and the input to the new faded up across the block. All subsequent
         * blocks are passed through the new filter.
         * - Mixing the output of the old and new filters for the next
         * num_blocks blocks.
         */
        void crossfade_filter(const Filter &filter);

        /** Crossfade to a zero-valued filter */
        void fade_down();

        /** Switch to a different filter at the start of the next block.
         */
        void set_filter(const Filter &filter);

        /** Switch to a zero-valued filter at the start of the next block. */
        void unset_filter();

       private:
        std::unique_ptr<block_convolver_impl::BlockConvolver> impl;
      };
    }  // namespace block_convolver
  }  // namespace dsp
}  // namespace ear
