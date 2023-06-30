
#include "ear/decorrelate.hpp"

#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <complex>
#include <random>
#include "ear/layout.hpp"
#include "kissfft.hh"

const double PI = boost::math::constants::pi<double>();

namespace ear {

  namespace {
    using RandEngine = std::mt19937;

    template <typename Engine>
    double genRandFloat(Engine &e) {
      return e() / static_cast<double>(0x100000000l);
    }
  }  // namespace

  /** @brief Design an all-pass random-phase FIR filter.
   *
   * @param decorrelator_id Random seed, to obtain different filters.
   * @param size filter length.
   *
   * @return  Filter coefficients.
   */
  std::vector<double> designDecorrelatorBasic(int decorrelatorId, int size) {
    RandEngine randEngine(decorrelatorId);
    std::vector<std::complex<double>> freqDomainData(size);
    freqDomainData[0] = std::complex<double>(1.0, 0.0);
    for (size_t i = 0; i < size / 2 - 1; ++i) {
      freqDomainData[i + 1] = std::exp(
          std::complex<double>(0.0, 2.0 * PI * genRandFloat(randEngine)));
    }
    freqDomainData[size / 2] = std::complex<double>(1.0, 0.0);
    for (size_t i = 0; i < freqDomainData.size() / 2; ++i) {
      freqDomainData[size / 2 + i] = std::conj(freqDomainData[size / 2 - i]);
    }
    kissfft<double> fft(size, true);
    std::vector<std::complex<double>> timeDomainData(size);
    fft.transform(&freqDomainData[0], &timeDomainData[0]);
    std::vector<double> timeDomainDataReal(size);
    for (size_t i = 0; i < timeDomainData.size(); ++i) {
      timeDomainDataReal[i] = timeDomainData[i].real() / size;
    }
    return timeDomainDataReal;
  }

  const int decorrelator_size = 512;

  template <>
  std::vector<double> designDecorrelator<double>(const Layout &layout,
                                                 size_t channelIdx) {
    auto &dec_channel_name = layout.channels().at(channelIdx).name();

    // rather than sorting the channel names, then searching that to get an
    // index, find what the index would be directly by scanning the channel
    // list once
    int dec_id = 0;
    for (auto &channel : layout.channels())
      if (channel.name() < dec_channel_name) dec_id++;

    return designDecorrelatorBasic(dec_id, decorrelator_size);
  }

  template <>
  std::vector<float> designDecorrelator<float>(const Layout &layout,
                                               size_t channelIdx) {
    auto decorrelator = designDecorrelator<double>(layout, channelIdx);

    std::vector<float> decorrelator_float(decorrelator.size());
    for (size_t sample_idx = 0; sample_idx < decorrelator.size(); sample_idx++)
      decorrelator_float[sample_idx] = (float)decorrelator[sample_idx];

    return decorrelator_float;
  }

  template <typename T>
  std::vector<std::vector<T>> designDecorrelators(const Layout &layout) {
    std::vector<std::vector<T>> decorrelators(layout.channels().size());
    for (size_t channel_idx = 0; channel_idx < layout.channels().size();
         channel_idx++)
      decorrelators[channel_idx] = designDecorrelator<T>(layout, channel_idx);

    return decorrelators;
  }

  template std::vector<std::vector<float>> EAR_EXPORT
  designDecorrelators(const Layout &layout);
  template std::vector<std::vector<double>> EAR_EXPORT
  designDecorrelators(const Layout &layout);

  int decorrelatorCompensationDelay() { return (decorrelator_size - 1) / 2; }
}  // namespace ear
