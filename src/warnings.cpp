#include "ear/warnings.hpp"
#include <cstdio>

namespace ear {
  void default_warning_cb_fn(const ear::Warning &warning) {
    fputs("libear: warning: ", stderr);
    fputs(warning.message.c_str(), stderr);
    fputc('\n', stderr);
  }

  const WarningCB default_warning_cb = default_warning_cb_fn;
}  // namespace ear
