#include "ear/dsp/block_convolver.hpp"
#include <boost/make_unique.hpp>
#include "block_convolver_impl.hpp"

namespace ear {
  namespace dsp {
    namespace block_convolver {
      Context::Context(size_t block_size, FFTImpl<real_t> &fft_impl)
          : impl(std::make_shared<block_convolver_impl::Context>(block_size,
                                                                 fft_impl)) {}

      Filter::Filter(const Context &ctx, size_t n, const real_t *filter)
          : impl(std::make_shared<block_convolver_impl::Filter>(ctx.impl, n,
                                                                filter)) {}

      size_t Filter::num_blocks() const { return impl->num_blocks(); }

      BlockConvolver::BlockConvolver(const Context &ctx, size_t num_blocks)
          : impl(boost::make_unique<block_convolver_impl::BlockConvolver>(
                ctx.impl, num_blocks)) {}

      BlockConvolver::BlockConvolver(const Context &ctx, const Filter &filter,
                                     size_t num_blocks)
          : impl(boost::make_unique<block_convolver_impl::BlockConvolver>(
                ctx.impl, filter.impl, num_blocks)) {}

      void BlockConvolver::crossfade_filter(const Filter &filter) {
        impl->crossfade_filter(filter.impl);
      }

      void BlockConvolver::fade_down() { impl->crossfade_filter(nullptr); }

      void BlockConvolver::set_filter(const Filter &filter) {
        impl->set_filter(filter.impl);
      }

      void BlockConvolver::unset_filter() { impl->set_filter(nullptr); }

      void BlockConvolver::process(const float *in, float *out) {
        impl->process(in, out);
      }

      BlockConvolver::~BlockConvolver() = default;
    }  // namespace block_convolver
  }  // namespace dsp
}  // namespace ear
