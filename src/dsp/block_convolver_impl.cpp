#include "block_convolver_impl.hpp"
#include <algorithm>
#include <vector>
#include "ear/fft.hpp"
#include "ear/helpers/assert.hpp"

namespace ear {
  namespace dsp {
    namespace block_convolver_impl {
      Context::Context(size_t block_size, FFTImpl<real_t> &fft_impl)
          : block_size(block_size),
            fft(fft_impl.plan(2 * block_size)),
            td_size(2 * block_size),
            fd_size(block_size + 1) {}

      Filter::Filter(const std::shared_ptr<Context> &ctx,
                     const TDVector &filter) {
        TDVector td = TDVector(ctx->td_size);
        auto fft_work_buf = ctx->fft->alloc_workbuf();

        for (size_t offset = 0; offset < (size_t)filter.size();
             offset += ctx->block_size) {
          size_t this_block_size =
              std::min(ctx->block_size, filter.size() - offset);

          // copy to the first half (or less) of td, and zero the second half
          td.setZero();
          td(Eigen::seqN(0, this_block_size)) =
              filter(Eigen::seqN(offset, this_block_size));

          // fft into fd
          FDVector fd = FDVector(ctx->fd_size);
          ctx->fft->transform_forward(td.data(), fd.data(), *fft_work_buf);

          blocks.emplace_back(std::move(fd));
        }
      }

      Filter::Filter(const std::shared_ptr<Context> &ctx, size_t n,
                     const real_t *filter)
          : Filter(ctx, TDVector::Map(filter, n)) {}

      BlockConvolver::BlockConvolver(const std::shared_ptr<Context> &ctx,
                                     size_t num_blocks)
          : ctx(ctx),
            fft_work_buf(ctx->fft->alloc_workbuf()),
            num_blocks(num_blocks),
            filter_queue(num_blocks + 1, NULL),
            filter_ofs(0),
            spectra_ofs(0),
            last_tail(ctx->block_size),
            current_td_old(ctx->td_size),
            current_td_new(ctx->td_size),
            multiply_out(ctx->fd_size),
            out_td(ctx->td_size) {
        for (size_t i = 0; i < num_blocks; i++) {
          spectra_queue_old.emplace_back(ctx->fd_size);
          spectra_queue_new.emplace_back(ctx->fd_size);
        }
      }

      // num_blocks = 0 by default; if so use num_blocks from filter.
      BlockConvolver::BlockConvolver(
          const std::shared_ptr<Context> &ctx,
          const std::shared_ptr<const Filter> &filter, size_t num_blocks)
          : BlockConvolver(ctx,
                           num_blocks > 0 ? num_blocks : filter->num_blocks()) {
        set_filter(filter);
      }

      void BlockConvolver::crossfade_filter(
          const std::shared_ptr<const Filter> &filter) {
        check_filter(filter);

        filters(0) = filter;
      }

      void BlockConvolver::set_filter(
          const std::shared_ptr<const Filter> &filter) {
        check_filter(filter);

        for (auto &filter_ref : filter_queue) filter_ref = filter;
      }

      void BlockConvolver::check_filter(
          const std::shared_ptr<const Filter> &filter) {
        if (filter) {
          for (auto &block : filter->blocks)
            if ((size_t)block.size() != ctx->fd_size) {
              throw invalid_argument(
                  "Filter block size is not equal to BlockConvolver block "
                  "size; "
                  "was this created using the same context?");
            }
          if (filter->num_blocks() > num_blocks)
            throw invalid_argument("too many blocks in given Filter");
        }
      }

      std::shared_ptr<const Filter> &BlockConvolver::filters(size_t i) {
        return filter_queue[(filter_ofs + i) % (num_blocks + 1)];
      }

      BlockConvolver::ZeroTrack<FDVector> &BlockConvolver::spectra_old(
          size_t i) {
        return spectra_queue_old[(spectra_ofs + i) % num_blocks];
      }

      BlockConvolver::ZeroTrack<FDVector> &BlockConvolver::spectra_new(
          size_t i) {
        return spectra_queue_new[(spectra_ofs + i) % num_blocks];
      }

      void BlockConvolver::rotate_queues() {
        // older blocks are at higher indices, so move spectra_ofs and
        // filter_ofs left, with wrap-around
        spectra_ofs = (spectra_ofs + num_blocks - 1) % num_blocks;
        filter_ofs = (filter_ofs + (num_blocks + 1) - 1) % (num_blocks + 1);

        // By default the next filter to use is the previous.
        filters(0) = filters(1);
      }

      /** Produce two versions of the block of n samples in in, one faded down
       * across the block, and the other faded up across the block.
       */
      void fade_down_and_up(const Eigen::Ref<const TDVector> &in,
                            Eigen::Ref<TDVector> down,
                            Eigen::Ref<TDVector> up) {
        ear_assert(in.size() <= down.size(), "down is incorrect size");
        ear_assert(in.size() <= up.size(), "up is incorrect size");

        real_t i_scale = 1.0 / in.size();

        for (int i = 0; i < in.size(); i++) {
          real_t a_v = (real_t)i * i_scale;
          real_t b_v = 1.0f - a_v;
          up[i] = a_v * in[i];
          down[i] = b_v * in[i];
        }
      }

      void BlockConvolver::process(const Eigen::Ref<const TDVector> &in,
                                   Eigen::Ref<TDVector> out) {
        if (in.data() != nullptr && (size_t)in.size() != ctx->block_size)
          throw invalid_argument(
              "in must be a null pointer or of size block_size");
        if ((size_t)out.size() != ctx->block_size)
          throw invalid_argument("out must be of size block_size");

        auto first_half = Eigen::seqN(0, ctx->block_size);
        auto second_half = Eigen::seqN(ctx->block_size, ctx->block_size);

        // Pad and fft in into spectra_queue_old and spectra_queue_new, fading
        // if necessary.
        if (in.data() == nullptr || (in.array() == 0.0).all()) {
          // zero if input is zero
          spectra_old(0).clear();
          spectra_new(0).clear();
        } else {
          // has the filter changed?
          if (filters(1) != filters(0)) {
            // if so, produce a version fading down in current_td_old and up in
            // current_td_new, then fft both
            fade_down_and_up(in, current_td_old.write(),
                             current_td_new.write());
            // fft, and clear the second half as this may have been modified
            ctx->fft->transform_forward(current_td_old.write().data(),
                                        spectra_old(0).write().data(),
                                        *fft_work_buf);
            current_td_old.write()(second_half).setZero();

            ctx->fft->transform_forward(current_td_new.write().data(),
                                        spectra_new(0).write().data(),
                                        *fft_work_buf);
            current_td_new.write()(second_half).setZero();
          } else {
            // otherwise just fft directly to new spectra.
            current_td_new.write()(first_half) = in;
            // fft, and clear the second half as this may have been modified
            ctx->fft->transform_forward(current_td_new.write().data(),
                                        spectra_new(0).write().data(),
                                        *fft_work_buf);
            current_td_new.write()(second_half).setZero();
            // all in spectra_new for this block
            spectra_old(0).clear();
          }
        }

        // Multiply the spectra of all filters with the corresponding input
        // spectra, summing into multiply_out. Blocks in spectra_queue_new are
        // multiplied with the current filter block, and spectra_queue_old with
        // the previous filter block.
        multiply_out.clear();
        for (size_t i = 0; i < num_blocks; i++) {
          const Filter *old_filter = filters(i + 1).get();
          const Filter *new_filter = filters(i).get();

          if (old_filter != NULL && i < old_filter->blocks.size() &&
              !spectra_old(i).zero) {
            multiply_out.write() +=
                old_filter->blocks[i].cwiseProduct(spectra_old(i).read());
          }
          if (new_filter != NULL && i < new_filter->blocks.size() &&
              !spectra_new(i).zero) {
            multiply_out.write() +=
                new_filter->blocks[i].cwiseProduct(spectra_new(i).read());
          }
        }

        // scale factor for normalising the result of fft followed by inverse
        real_t norm = 1.0 / (real_t)(2 * ctx->block_size);

        // Inverse fft, then send the first half plus the last tail to the
        // output, and write the second half to last_tail, to be used in the
        // next block.
        if (!multiply_out.zero) {
          ctx->fft->transform_reverse(multiply_out.write().data(),
                                      out_td.write().data(), *fft_work_buf);

          // Mix last_tail into the first half of out_td, and replace last_tail
          // with the second half of out_td.
          if (!last_tail.zero) out_td.write()(first_half) += last_tail.read();
          last_tail.write() = out_td.read()(second_half);

          out = out_td.read()(first_half) * norm;
        } else if (!last_tail.zero) {
          // no spectra, just send the last tail if nonzero
          out = last_tail.read() * norm;
          last_tail.clear();
        } else {
          // no spectra or last tail, zero the output
          out.setZero();
        }

        rotate_queues();
      }

      void BlockConvolver::process(const float *in, float *out) {
        auto in_map = TDVector::Map(in, ctx->block_size);
        auto out_map = TDVector::Map(out, ctx->block_size);
        process(in_map, out_map);
      }

    }  // namespace block_convolver_impl
  }  // namespace dsp
}  // namespace ear
