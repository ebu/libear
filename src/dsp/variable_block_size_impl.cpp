#include <Eigen/Core>
#include <boost/make_unique.hpp>
#include <functional>
#include "ear/dsp/ptr_adapter.hpp"
#include "ear/dsp/variable_block_size.hpp"
#include "ear/helpers/assert.hpp"

namespace ear {
  namespace dsp {
    /// Adapt something that processes fixed-size blocks of samples into one
    /// that processes variable sized blocks by adding some delay.
    ///
    /// This can be used with e.g. BlockConvolver to process arbitrary block
    /// lengths. This isn't built into the BlockConvolver, because it's not
    /// always necessary, and because this introduces some delay; if we adapt
    /// multiple components with this then we can save some delay compared to
    /// having it built into each component.
    class VariableBlockSizeAdapterImpl {
     public:
      using Samples = Eigen::MatrixXf;
      using ProcessFunc = VariableBlockSizeAdapter::ProcessFunc;

      /// \param block_size number of samples accepted by \p process_func
      /// \param num_channels_in number of input channels
      /// \param num_channels_out number of output channels
      /// \param process_func function to call to process block_size samples
      VariableBlockSizeAdapterImpl(size_t block_size, size_t num_channels_in,
                                   size_t num_channels_out,
                                   std::function<ProcessFunc> process_func)
          : process_func(process_func),
            block_size(block_size),
            input_buffer(Samples::Zero(block_size, num_channels_in)),
            output_buffer(Samples::Zero(block_size, num_channels_out)),
            samples_in_input(0),
            input_buffer_ptr(num_channels_in),
            output_buffer_ptr(num_channels_out) {
        input_buffer_ptr.set_eigen(input_buffer);
        output_buffer_ptr.set_eigen(output_buffer);
      }

      /// Process n samples.
      /// \param in input samples, n rows and num_channels_in cols
      /// \param out output samples, n rows and num_channels_out cols
      void process(size_t nsamples_in, const float* const* in,
                   float* const* out) {
        Eigen::Index nsamples = nsamples_in;
        Eigen::Index sample = 0;
        while (sample < nsamples) {
          // move in -> input_buffer and out -> output_buffer until out of
          // samples (or input_buffer is full and output_buffer is empty)
          Eigen::Index to_transfer =
              std::min(nsamples - sample, block_size - samples_in_input);

          for (Eigen::Index in_channel = 0; in_channel < input_buffer.cols();
               in_channel++)
            for (Eigen::Index i = 0; i < to_transfer; i++)
              input_buffer(samples_in_input + i, in_channel) =
                  in[in_channel][sample + i];

          for (Eigen::Index out_channel = 0; out_channel < output_buffer.cols();
               out_channel++)
            for (Eigen::Index i = 0; i < to_transfer; i++)
              out[out_channel][sample + i] =
                  output_buffer(samples_in_input + i, out_channel);

          sample += to_transfer;
          samples_in_input += to_transfer;

          // run process from input_buffer to output_buffer
          bool run_process = samples_in_input == block_size;
          if (run_process) {
            process_func(input_buffer_ptr.ptrs(), output_buffer_ptr.ptrs());
            samples_in_input = 0;
          }

          // check that we made progress
          ear_assert(run_process || to_transfer > 0, "no progress made");
        }

        ear_assert(sample == nsamples, "processed more samples than expected");
      }

      /// The delay introduced by the variable block size processing, not
      /// accounting for any delay introduced by the inner process.
      Eigen::Index get_delay() const { return block_size; }

     private:
      std::function<ProcessFunc> process_func;
      Eigen::Index block_size;
      // Buffers for input and output samples, both block_size long.
      // input_buffer contains samples_in_input samples, starting at the start.
      // output_buffer contains (block_size - samples_in_input) samples,
      // starting at samples_in_input (so, aligned at the end of the buffer).
      Samples input_buffer;
      Samples output_buffer;
      Eigen::Index samples_in_input;

      PtrAdapter input_buffer_ptr;
      PtrAdapter output_buffer_ptr;
    };

    VariableBlockSizeAdapter::VariableBlockSizeAdapter(
        size_t block_size, size_t num_channels_in, size_t num_channels_out,
        std::function<ProcessFunc> process_func)
        : impl(boost::make_unique<VariableBlockSizeAdapterImpl>(
              block_size, num_channels_in, num_channels_out, process_func)) {}

    void VariableBlockSizeAdapter::process(size_t nsamples,
                                           const float* const* in,
                                           float* const* out) {
      impl->process(nsamples, in, out);
    }

    /// The delay introduced by the variable block size processing, not
    /// accounting for any delay introduced by the inner process.
    int VariableBlockSizeAdapter::get_delay() const {
      return impl->get_delay();
    }

    VariableBlockSizeAdapter::~VariableBlockSizeAdapter() = default;
  }  // namespace dsp
}  // namespace ear
