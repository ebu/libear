#pragma once
#include "export.hpp"
#include "layout.hpp"

namespace ear {

  /// Get all ITU-R BS.2051 layouts.
  EAR_EXPORT const std::vector<Layout>& loadLayouts();

  /// Get a layout given its ITU-R BS.2051 name (e.g. `4+5+0`).
  EAR_EXPORT Layout getLayout(const std::string& name);

}  // namespace ear
