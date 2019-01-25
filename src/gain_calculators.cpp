#include "ear/gain_calculators.hpp"

#include "ear/common/helpers/make_unique.hpp"
#include "ear/direct_speakers/gain_calculator_direct_speakers.hpp"
#include "ear/hoa/gain_calculator_hoa.hpp"
#include "ear/object_based/gain_calculator_objects.hpp"

namespace ear {

  // DirectSpeakers

  GainCalculatorDirectSpeakers::~GainCalculatorDirectSpeakers() = default;

  GainCalculatorDirectSpeakers::GainCalculatorDirectSpeakers(
      const Layout& layout,
      std::map<std::string, std::string> additionalSubstitutions)
      : _impl(std::make_unique<GainCalculatorDirectSpeakersImpl>(
            layout, additionalSubstitutions)){};

  template <typename T>
  void GainCalculatorDirectSpeakers::calculate(
      const DirectSpeakersTypeMetadata& metadata, std::vector<T>& gains,
      const WarningCB& warning_cb) {
    _impl->calculate(metadata, gains, warning_cb);
  };

  template EAR_EXPORT void GainCalculatorDirectSpeakers::calculate<double>(
      const DirectSpeakersTypeMetadata&, std::vector<double>&,
      const WarningCB&);
  template EAR_EXPORT void GainCalculatorDirectSpeakers::calculate<float>(
      const DirectSpeakersTypeMetadata&, std::vector<float>&, const WarningCB&);

  // Objects

  GainCalculatorObjects::~GainCalculatorObjects() = default;

  GainCalculatorObjects::GainCalculatorObjects(const Layout& layout)
      : _impl(std::make_unique<GainCalculatorObjectsImpl>(layout)){};

  template <typename T>
  void GainCalculatorObjects::calculate(const ObjectsTypeMetadata& metadata,
                                        std::vector<T>& directGains,
                                        std::vector<T>& diffuseGains,
                                        const WarningCB& warning_cb) {
    _impl->calculate(metadata, directGains, diffuseGains, warning_cb);
  };

  template EAR_EXPORT void GainCalculatorObjects::calculate<double>(
      const ObjectsTypeMetadata&, std::vector<double>&, std::vector<double>&,
      const WarningCB&);
  template EAR_EXPORT void GainCalculatorObjects::calculate<float>(
      const ObjectsTypeMetadata&, std::vector<float>&, std::vector<float>&,
      const WarningCB&);

  // HOA

  GainCalculatorHOA::~GainCalculatorHOA() = default;

  GainCalculatorHOA::GainCalculatorHOA(const Layout& layout)
      : _impl(std::make_unique<GainCalculatorHOAImpl>(layout)){};

  template <typename T>
  void GainCalculatorHOA::calculate(const HOATypeMetadata& metadata,
                                    std::vector<std::vector<T>>& gains,
                                    const WarningCB& warning_cb) {
    _impl->calculate(metadata, gains, warning_cb);
  };

  template EAR_EXPORT void GainCalculatorHOA::calculate<double>(
      const HOATypeMetadata&, std::vector<std::vector<double>>&,
      const WarningCB&);
  template EAR_EXPORT void GainCalculatorHOA::calculate<float>(
      const HOATypeMetadata&, std::vector<std::vector<float>>&,
      const WarningCB&);

}  // namespace ear
