#pragma once
#include <vector>
#include "export.hpp"
#include "layout.hpp"

namespace ear {
  /** @brief Design one filter for each channel in layout.
   *
   * @param layout Layout to design for; channel names are used to allocate
   * filters to channels.
   *
   * @return Decorrelation filters.
   */
  template <typename T = float>
  EAR_EXPORT std::vector<std::vector<T>> designDecorrelators(Layout layout);

  /** @brief Get the delay length needed to compensate for decorrelators
   *
   * @return Delay length in samples.
   */
  EAR_EXPORT int decorrelatorCompensationDelay();
}  // namespace ear
