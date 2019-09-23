#pragma once
#include <memory>
#include "../exceptions.hpp"
#include "ear/export.h"

namespace ear {
  namespace dsp {

    class DelayBufferImpl;

    /// A multi-channel delay buffer
    class EAR_EXPORT DelayBuffer {
     public:
      /// @param nchannels number of input and output channels
      /// @param nsamples length of the delay
      DelayBuffer(size_t nchannels, size_t nsamples);

      /// Process an arbitrary number of samples. \p input and \p output have
      /// \c nchannels channels and \p nsamples samples.
      void process(size_t nsamples, const float *const *input,
                   float *const *output);

      /// Get the delay in samples.
      int get_delay() const;

      ~DelayBuffer();

     private:
      std::unique_ptr<DelayBufferImpl> impl;
    };

  }  // namespace dsp
}  // namespace ear
