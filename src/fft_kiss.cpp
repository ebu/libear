#include <complex>
#include <cstddef>
#include <memory>
#include <vector>
#include "ear/fft.hpp"
#include "ear/helpers/assert.hpp"
#include "kissfft/kissfft.hh"

namespace ear {

  // KISS implementation of FFT interface
  //
  // KISS doesn't provide exactly what we need to implement the API, so the
  // implementation is a bit funky. Note that this isn't a problem with the API
  // -- FFTW and IPP should both fit nicely and require fewer copies etc.
  //
  // - kiffsst objects contain both the plan and working buffers, so have to be
  //   held in the FFTWorkBuf implementation to make it thread safe. These are
  //   also stored in the plan object so that they can at least be copied into
  //   the FFTWorkBuf to save on initialisation time.
  //
  // - KISS only supports r2c real transforms, not c2r, so transform_reverse
  //   performs a full transform with some copying back and forth to temporary
  //   buffers stored in the FFTWorkBuf.
  //
  // - KISS transform_real packs the real N/2 component into the imaginary part
  //   of the first component -- this is unpacked so that all FFT
  //   implementations can use the same format.
  //
  // - KISS transform_real only supports even input lengths. This is made part
  //   of the interface as it's a sensible requirement in general.

  template <typename Real>
  class WorkBufKiss : public FFTWorkBuf {
   public:
    WorkBufKiss(size_t n_fft, kissfft<Real> plan_forward,
                kissfft<Real> plan_reverse)
        : plan_forward(plan_forward),
          plan_reverse(plan_reverse),
          reverse_tmp_input(n_fft),
          reverse_tmp_output(n_fft) {}

   public:
    kissfft<Real> plan_forward;
    kissfft<Real> plan_reverse;
    std::vector<std::complex<Real>> reverse_tmp_input;
    std::vector<std::complex<Real>> reverse_tmp_output;
  };

  template <typename Real>
  class FFTPlanKiss : public FFTPlan<Real> {
   public:
    using Complex = typename FFTPlan<Real>::Complex;

    FFTPlanKiss(size_t n_fft)
        : n_fft(n_fft),
          plan_forward(n_fft / 2, false),
          plan_reverse(n_fft, true) {}

    void transform_forward(Real *input, Complex *output,
                           FFTWorkBuf &workbuf) const override {
      WorkBufKiss<Real> &workbuf_kiss =
          dynamic_cast<WorkBufKiss<Real> &>(workbuf);
      workbuf_kiss.plan_forward.transform_real(input, output);

      // kiss transform_real packs the n/2+1 real term into the 0th imaginary
      // term; put it in its proper place
      output[n_fft / 2] = output[0].imag();
      output[0].imag(0.0);
    }

    void transform_reverse(Complex *input, Real *output,
                           FFTWorkBuf &workbuf) const override {
      WorkBufKiss<Real> &workbuf_kiss =
          dynamic_cast<WorkBufKiss<Real> &>(workbuf);

      for (size_t i = 0; i < n_fft; i++)
        workbuf_kiss.reverse_tmp_input[i] =
            i < n_fft / 2 + 1 ? input[i] : std::conj(input[n_fft - i]);

      workbuf_kiss.plan_reverse.transform(
          workbuf_kiss.reverse_tmp_input.data(),
          workbuf_kiss.reverse_tmp_output.data());

      for (size_t i = 0; i < n_fft; i++)
        output[i] = workbuf_kiss.reverse_tmp_output[i].real();
    }

    std::unique_ptr<FFTWorkBuf> alloc_workbuf() const override {
      return std::unique_ptr<FFTWorkBuf>(
          new WorkBufKiss<Real>(n_fft, plan_forward, plan_reverse));
    }

   private:
    size_t n_fft;
    kissfft<Real> plan_forward;
    kissfft<Real> plan_reverse;
  };

  template <typename Real>
  class FFTKiss : public FFTImpl<Real> {
   public:
    std::shared_ptr<FFTPlan<Real>> plan(size_t n_fft) const override {
      ear_assert(n_fft % 2 == 0, "n_fft must be even");
      return std::make_shared<FFTPlanKiss<Real>>(n_fft);
    }
  };

  template <typename Real>
  FFTImpl<Real> &get_fft_kiss() {
    static FFTKiss<Real> fft;
    return fft;
  }

  template FFTImpl<float> &get_fft_kiss<float>();
  template FFTImpl<double> &get_fft_kiss<double>();

}  // namespace ear
