#pragma once
#include <complex>
#include <memory>

namespace ear {
  /// \file
  /// Interface for performing c2r and r2c FFTs with pluggable implementations.

  /// temporary buffers needed to perform an FFT; allocated by calling
  /// FFTPlan::alloc_workbuf.
  ///
  /// As this contains data which is mutated by the transform functions, this
  /// must not be shared between threads.
  class FFTWorkBuf {
   public:
    virtual ~FFTWorkBuf() = 0;
  };

  inline FFTWorkBuf::~FFTWorkBuf() {}

  /// Plan for performing an FFT of a particular size/layout/type; allocated by
  /// calling FFTImpl::plan
  ///
  /// This is not mutated when transform_* are called, so one plan may be shared
  /// between threads.
  template <typename Real>
  class FFTPlan {
   public:
    using Complex = std::complex<Real>;
    /// Execute an r2c forwards transform.
    /// \param input n_fft input samples
    /// \param output n_fft/2+1 output samples containing the first half of the
    ///     complex frequency components without any packing.
    /// \param workbuf temporary buffers allocated with alloc_workbuf
    virtual void transform_forward(Real *input, Complex *output,
                                   FFTWorkBuf &workbuf) const = 0;

    /// Execute an c2r inverse transform.
    /// \param input n_fft/2+1 input samples, in the same format as given by
    ///     transform_forward
    /// \param output n_fft output samples
    /// \param workbuf temporary buffers allocated with alloc_workbuf
    virtual void transform_reverse(Complex *input, Real *output,
                                   FFTWorkBuf &workbuf) const = 0;

    /// allocate temporary buffers to be used with transform_*
    virtual std::unique_ptr<FFTWorkBuf> alloc_workbuf() const = 0;

    virtual ~FFTPlan() {}
  };

  /// An FFT implementation.
  template <typename Real>
  class FFTImpl {
   public:
    /// Plan to execute an r2c/c2r FFT using this implementation.
    /// \param n_fft number of points in the real parts; i.e. the input to
    ///     transform_forward and the output of transform_reverse. Must be even.
    virtual std::shared_ptr<FFTPlan<Real>> plan(size_t n_fft) const = 0;

    virtual ~FFTImpl() {}
  };

  /// Get a KISS FFT implementation for a particular type. This is always
  /// available.
  template <typename Real>
  FFTImpl<Real> &get_fft_kiss();

}  // namespace ear
