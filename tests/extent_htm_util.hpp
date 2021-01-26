#pragma once

#include <boost/make_unique.hpp>
#include <memory>
#include <string>

#include "ear/bs2051.hpp"
#include "object_based/extent.hpp"
#include "object_based/extent_htm.hpp"

namespace ear {
  /// make an extent panner which either uses HTM or the plain spreading panner
  /// this bypasses the default selection in PolarExtentPanner, which may depend
  /// on the configuration
  static PolarExtentPanner makeExtentPanner(const std::string &layoutName,
                                            bool htm = false, int levels = 0) {
    Layout layout = getLayout(layoutName).withoutLfe();
    std::shared_ptr<PointSourcePanner> psp = configurePolarPanner(layout);

    if (htm)
      return {psp, boost::make_unique<SpreadingPannerHTM>(
                       psp, PolarExtentPanner::nRowsDefault, levels)};
    else
      return {psp, boost::make_unique<SpreadingPanner>(
                       psp, PolarExtentPanner::nRowsDefault)};
  }
}  // namespace ear
