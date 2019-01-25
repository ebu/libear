#include "ear/screen.hpp"

namespace ear {
  Screen EAR_EXPORT getDefaultScreen() {
    return PolarScreen{1.78, PolarPosition(0.0, 0.0, 1.0), 58.0};
  }
}  // namespace ear
