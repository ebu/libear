#pragma once
#include <functional>
#include <memory>
#include "ear/export.h"

namespace ear {
  namespace dsp {
    class VariableBlockSizeAdapterImpl;
    /// Adapt something that processes fixed-size blocks of samples into one
    /// that processes variable sized blocks by adding some delay.
    ///
    /// This can be used with e.g. BlockConvolver to process arbitrary block
    /// lengths. This isn't built into the BlockConvolver, because it's not
    /// always necessary, and because this introduces some delay; if we adapt
    /// multiple components with this then we can save some delay compared to
    /// having it built into each component.
    class EAR_EXPORT VariableBlockSizeAdapter {
     public:
      using ProcessFunc = void(const float* const* in, float* const* out);

      /// \param block_size number of samples accepted by \p process_func
      /// \param num_channels_in number of input channels
      /// \param num_channels_out number of output channels
      /// \param process_func function to call to process block_size samples
      VariableBlockSizeAdapter(size_t block_size, size_t num_channels_in,
                               size_t num_channels_out,
                               std::function<ProcessFunc> process_func);

      /// Process nsamples samples.
      void process(size_t nsamples, const float* const* in, float* const* out);

      /// The delay introduced by the variable block size processing, not
      /// accounting for any delay introduced by the inner process.
      int get_delay() const;

      ~VariableBlockSizeAdapter();

     private:
      std::unique_ptr<VariableBlockSizeAdapterImpl> impl;
    };
  }  // namespace dsp
}  // namespace ear
