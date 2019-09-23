#pragma once
#include <boost/optional.hpp>
#include "ear/screen.hpp"

namespace ear {
  class ScreenScaleHandler {
   public:
    ScreenScaleHandler(boost::optional<Screen> reproductionScreen)
        : _reproductionScreen(reproductionScreen){};

   private:
    boost::optional<Screen> _reproductionScreen;
  };
}  // namespace ear
