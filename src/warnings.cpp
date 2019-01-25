#include "ear/warnings.hpp"
#include <iostream>

namespace ear {
  void default_warning_cb_fn(const ear::Warning &warning) {
    std::cerr << "libear: warning: " << warning.message << std::endl;
  }

  const WarningCB default_warning_cb = default_warning_cb_fn;
}  // namespace ear
