#include <Eigen/Core>
#include <boost/make_unique.hpp>
#include "ear/dsp/delay_buffer.hpp"
#include "ear/exceptions.hpp"

namespace ear {
  namespace dsp {

    /// A multi-channel delay buffer, templated over the type of sample.
    class DelayBufferImpl {
     public:
      /// @param nchannels number of input and output channels
      /// @param nsamples length of the delay
      DelayBufferImpl(size_t nchannels, size_t nsamples)
          : delaymem(Eigen::MatrixXf::Zero(nsamples, nchannels)) {}

      /// Process an arbitrary number of samples. \p input and \p output have
      /// \c nchannels channels and \p nsamples samples.
      void process(size_t nsamples, const float *const *input,
                   float *const *output) {
        Eigen::Index nchannels = delaymem.cols();
        Eigen::Index delay = delaymem.rows();

        for (Eigen::Index channel = 0; channel < nchannels; channel++) {
          for (Eigen::Index sample = 0; sample < nsamples + delay; sample++) {
            // transfer from:
            //    [delaymem; input]
            // to:
            //    [output; delaymem]
            float value = sample < delay ? delaymem(sample, channel)
                                         : input[channel][sample - delay];

            if (sample < nsamples)
              output[channel][sample] = value;
            else
              delaymem(sample - nsamples, channel) = value;
          }
        }
      }

      /// Get the delay in samples.
      int get_delay() const { return (int)delaymem.rows(); }

     private:
      Eigen::MatrixXf delaymem;
    };

    DelayBuffer::DelayBuffer(size_t nchannels, size_t nsamples)
        : impl(boost::make_unique<DelayBufferImpl>(nchannels, nsamples)) {}

    void DelayBuffer::process(size_t nsamples, const float *const *input,
                              float *const *output) {
      impl->process(nsamples, input, output);
    }

    int DelayBuffer::get_delay() const { return impl->get_delay(); }

    DelayBuffer::~DelayBuffer() = default;

  }  // namespace dsp
}  // namespace ear
