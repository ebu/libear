#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "export.hpp"
#include "layout.hpp"
#include "metadata.hpp"
#include "warnings.hpp"

namespace ear {

  class GainCalculatorDirectSpeakersImpl;
  class GainCalculatorObjectsImpl;
  class GainCalculatorHOAImpl;

  /// Gain calculator for typeDefinition == "DirectSpeakers"
  class EAR_EXPORT GainCalculatorDirectSpeakers {
   public:
    GainCalculatorDirectSpeakers(
        const Layout& layout,
        std::map<std::string, std::string> additionalSubstitutions = {});
    ~GainCalculatorDirectSpeakers();

    /// Calculate gains for metadata. \p gains contains per-loudspeaker
    /// gains to render this channel.
    template <typename T>
    void calculate(const DirectSpeakersTypeMetadata& metadata,
                   std::vector<T>& gains,
                   const WarningCB& warning_cb = default_warning_cb);

   private:
    std::unique_ptr<GainCalculatorDirectSpeakersImpl> _impl;
  };

  /// Gain calculator for typeDefinition == "Objects"
  class EAR_EXPORT GainCalculatorObjects {
   public:
    GainCalculatorObjects(const Layout& layout);
    ~GainCalculatorObjects();

    /// Calculate gains for metadata. \p directGains and \p diffuseGains
    /// contains per-loudspeaker gains to render this channel.
    ///
    /// To apply these gains:
    /// - \p directGains are applied to this channel, and summed with other
    ///   objects into a n-channel direct bus
    /// - \p diffuseGains are applied to this channel, and summed with other
    ///   objects into a n-channel diffuse bus
    /// - each channel in the diffuse bus is processed with the corresponding
    ///   FIR filter given by designDecorrelators()
    /// - each channel in the direct bus is delayed by
    ///   decorrelatorCompensationDelay() samples to compensate for the delay
    ///   through the decorrelation filters
    /// - the output of the decorrelation filters and delays are mixed together
    ///   to form the output
    template <typename T>
    void calculate(const ObjectsTypeMetadata& metadata,
                   std::vector<T>& directGains, std::vector<T>& diffuseGains,
                   const WarningCB& warning_cb = default_warning_cb);

   private:
    std::unique_ptr<GainCalculatorObjectsImpl> _impl;
  };

  /// Gain calculator for typeDefinition == "HOA"
  class EAR_EXPORT GainCalculatorHOA {
   public:
    GainCalculatorHOA(const Layout& layout);
    ~GainCalculatorHOA();

    /// Calculate a decode matrix for metadata.
    /// Gains contains one vector of per-loudspeaker gains per
    /// input channel, and must be the right size before calling.
    template <typename T>
    void calculate(const HOATypeMetadata& metadata,
                   std::vector<std::vector<T>>& gains,
                   const WarningCB& warning_cb = default_warning_cb);

   private:
    std::unique_ptr<GainCalculatorHOAImpl> _impl;
  };

}  // namespace ear
