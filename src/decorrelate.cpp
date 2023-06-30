
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
  std::vector<std::vector<double>> designDecorrelators<double>(
      const Layout &layout) {
    std::vector<std::string> channelNames = layout.channelNames();
    std::vector<std::string> channelNamesSorted(channelNames);
    std::sort(channelNamesSorted.begin(), channelNamesSorted.end());
    std::vector<std::vector<double>> decorrelators;
    for (auto channelName : channelNames) {
      auto it =
          std::find_if(channelNamesSorted.begin(), channelNamesSorted.end(),
                       [&channelName](const std::string name) -> bool {
                         return channelName == name;
                       });
      int index =
          static_cast<int>(std::distance(channelNamesSorted.begin(), it));
      std::vector<double> coefficients =
          designDecorrelatorBasic(index, decorrelator_size);
      decorrelators.push_back(coefficients);
    }
    return decorrelators;
  }

  template <>
  EAR_EXPORT std::vector<std::vector<float>> designDecorrelators<float>(
      const Layout &layout) {
    auto decorrelators = designDecorrelators<double>(layout);
    std::vector<std::vector<float>> decorrelators_float;

    for (auto &decorrelator : decorrelators) {
      std::vector<float> decorrelator_float(decorrelator.size());

      for (size_t i = 0; i < decorrelator.size(); i++)
        decorrelator_float[i] = (float)decorrelator[i];

      decorrelators_float.emplace_back(std::move(decorrelator_float));
    }

    return decorrelators_float;
  }

  int decorrelatorCompensationDelay() { return (decorrelator_size - 1) / 2; }
}  // namespace ear
