#pragma once

#include <Eigen/Core>
#include <complex>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>
#include "ear/fft.hpp"

namespace ear {
  namespace dsp {

    namespace block_convolver_impl {
      /// Type for real data (float).
      using real_t = float;
      /// Type for complex data.
      using complex_t = std::complex<real_t>;

      using TDVector = Eigen::Matrix<real_t, Eigen::Dynamic, 1>;
      using FDVector = Eigen::Matrix<complex_t, Eigen::Dynamic, 1>;

      /** Static data required to perform convolution of a particular block
       * size; may be shared between any number of BlockConvolver and
       * BlockConvolver::Filter instances. */
      class Context {
       public:
        /** Create a Context with a given block size.
         * @param block_size Block size in samples.
         * @param fft_impl FFT implementation to use.
         */
        Context(size_t block_size, FFTImpl<real_t> &fft_impl);

       private:
        // number of samples in a block
        const size_t block_size;

        // fft of block_size * 2
        std::shared_ptr<FFTPlan<real_t>> fft;

        // time domain fft block size
        const size_t td_size;
        // frequency domain fft block size
        const size_t fd_size;

        friend class BlockConvolver;
        friend class Filter;
      };

      /** A filter response which may be shared between many BlockConvolver
       * instances.
       *
       * This stores the pre-transformed filter blocks. */
      class Filter {
       public:
        Filter(const std::shared_ptr<Context> &ctx, const TDVector &filter);
        Filter(const std::shared_ptr<Context> &ctx, size_t n,
               const real_t *filter);

        /** The number of blocks in the filter. */
        size_t num_blocks() const { return blocks.size(); }

       private:
        std::vector<FDVector> blocks;

        friend class BlockConvolver;
      };

      /** BlockConvolver implements partitioned overlap-add convolution with a
       * fixed block size, with efficient fading between filters.
       */
      class BlockConvolver {
       public:
        /** Create a BlockConvolver given the block size and number of blocks.
         * @param ctx Context required for transformations.
         * @param num_blocks Maximum number of blocks of any filter used.
         */
        BlockConvolver(const std::shared_ptr<Context> &ctx, size_t num_blocks);

        /** Create a BlockConvolver given the block size and number of blocks.
         *  If filter == nullptr, num_blocks must be specified.
         * @param ctx Context required for transformations.
         * @param filter Initial filter to be used, or nullptr for no filter.
         * @param num_blocks Maximum number of blocks of any filter used; using
         * 0 will take the number of blocks from the passed filter.
         */
        BlockConvolver(const std::shared_ptr<Context> &ctx,
                       const std::shared_ptr<const Filter> &filter,
                       size_t num_blocks = 0);

        /** Pass a block of audio through the filter.
         * @param in Input samples of length block_size
         * @param out Output samples of length block_size
         */
        void process(const Eigen::Ref<const TDVector> &in,
                     Eigen::Ref<TDVector> out);

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
         *
         * @param filter Filter to crossfade to; should be alive for as long as
         * it is active. Pass nullptr for no filter.
         */
        void crossfade_filter(const std::shared_ptr<const Filter> &filter);

        /** Switch to a different filter at the start of the next block.
         * @param filter Filter to switch to; should be alive for as long as it
         * is active. Pass nullptr for no filter.
         */
        void set_filter(const std::shared_ptr<const Filter> &filter);

       private:
        // wrapper around an eigen type which knows if it contains only zeros
        template <typename T>
        struct ZeroTrack {
          template <typename... Args>
          ZeroTrack(Args &&... args) : data(std::forward<Args>(args)...) {
            clear();
          }
          T data;
          bool zero = false;
          /** Get access to the data for reading. */
          const T &read() { return data; }
          /** Get access to the data for writing; clears the zero flag. */
          T &write() {
            zero = false;
            return data;
          }
          /** Zero the buffer and set the zero flag. */
          void clear() {
            if (!zero) {
              zero = true;
              data.setZero();
            }
          }
        };

        std::shared_ptr<Context> ctx;
        std::unique_ptr<FFTWorkBuf> fft_work_buf;
        const size_t num_blocks;

        // check that a filter is valid for use with this convolver
        void check_filter(const std::shared_ptr<const Filter> &filter);

        // filters(i) accesses a circular buffer of filters, length num_blocks
        // + 1. on each frame, the input is crossfaded up and down, and passed
        // through
        // - filters[1:] (old)
        // - filters[:-1] (new)
        // After process, filters has been shifted one along, with filters(0)
        // left at filters(1); filters(0) is then set to filters(1), such that
        // the next filter is the same as the previous by default. set_filter
        // simply writes to filters(0).
        std::shared_ptr<const Filter> &filters(size_t i);
        // filter_queue and filter_ofs implement the above circular buffer.
        std::vector<std::shared_ptr<const Filter>> filter_queue;
        size_t filter_ofs;

        // a queue of spectra of the input, after zero padding on the right hand
        // side. If the filter is changed before the input block i frames ago,
        // spectra_old(i) contains the input faded down (to convolve with the
        // old filter) and spectra_new(i) contains the input faded up (to
        // convolve with the new filter). If the filter is not changed, only
        // spectra_new(i) contains data. num_blocks in length, each of size n+1.
        ZeroTrack<FDVector> &spectra_old(size_t i);
        ZeroTrack<FDVector> &spectra_new(size_t i);
        // implementation the above circular buffer
        std::vector<ZeroTrack<FDVector>> spectra_queue_old;
        std::vector<ZeroTrack<FDVector>> spectra_queue_new;
        size_t spectra_ofs;

        // Rotate the filter and spectra queues.
        void rotate_queues();

        // The second half of the ifft output for the last block, added to the
        // first half before output; size n.
        ZeroTrack<TDVector> last_tail;

        // Temporaries used by process to store the padded and faded inputs;
        // size 2n, the second half should always be zeros.
        ZeroTrack<TDVector> current_td_old;
        ZeroTrack<TDVector> current_td_new;
        // Temporary used by process to store the multiplied spectrum; size n+1.
        ZeroTrack<FDVector> multiply_out;
        // Temporary used to store the time domain output, size 2n.
        ZeroTrack<TDVector> out_td;
      };

    }  // namespace block_convolver_impl
  }  // namespace dsp
}  // namespace ear
