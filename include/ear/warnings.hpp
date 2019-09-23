#pragma once
#include <functional>
#include <string>
#include "export.hpp"

namespace ear {
  /// A warning message, containing a code and a corresponding message. The code
  /// does not need to be shown when displaying warnings; the message should
  /// contain all the information required, the code is just to allow
  /// implementations to take action on warnings without matching against the
  /// messages.
  struct Warning {
    enum class Code {
      /// LFE indication from frequency element does not match speakerLabel
      FREQ_SPEAKERLABEL_LFE_MISMATCH = 1,
      /// frequency indication present but does not indicate an LFE channel
      FREQ_NOT_LFE,
      /// frequency information is not implemented; ignoring
      FREQ_IGNORED,

      /// screenRef for HOA is not implemented; ignoring
      HOA_SCREENREF_NOT_IMPLEMENTED,
      /// nfcRefDist is not implemented; ignoring
      HOA_NFCREFDIST_NOT_IMPLEMENTED,
    };

    Code code;
    std::string message;
  };

  /// warning callback type; this is passed into `calculate` calls, and will be
  /// called with any warnings.
  using WarningCB = std::function<void(const Warning &warning)>;

  /// default warning callback which prints to stderr with the prefix `libear:
  /// warning: `
  extern EAR_EXPORT const WarningCB default_warning_cb;
}  // namespace ear
