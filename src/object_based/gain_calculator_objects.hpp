#pragma once
#include <memory>
#include "../common/channel_lock.hpp"
#include "../common/point_source_panner.hpp"
#include "../common/screen_edge_lock.hpp"
#include "../common/screen_scale.hpp"
#include "ear/common_types.hpp"
#include "ear/helpers/output_gains.hpp"
#include "ear/layout.hpp"
#include "ear/metadata.hpp"
#include "ear/warnings.hpp"
#include "extent.hpp"
#include "zone_exclusion.hpp"

namespace ear {

  class GainCalculatorObjectsImpl {
   public:
    GainCalculatorObjectsImpl(const Layout& layout);
    void calculate(const ObjectsTypeMetadata& metadata, OutputGains& direct,
                   OutputGains& diffuse,
                   const WarningCB& warning_cb = default_warning_cb);
    template <typename T>
    void calculate(const ObjectsTypeMetadata& metadata, std::vector<T>& direct,
                   std::vector<T>& diffuse,
                   const WarningCB& warning_cb = default_warning_cb) {
      OutputGainsT<T> direct_wrap(direct);
      OutputGainsT<T> diffuse_wrap(diffuse);

      calculate(metadata, direct_wrap, diffuse_wrap, warning_cb);
    }

   private:
    Layout _layout;
    std::shared_ptr<PointSourcePanner> _pointSourcePanner;
    ScreenEdgeLockHandler _screenEdgeLockHandler;
    ScreenScaleHandler _screenScaleHandler;
    ChannelLockHandler _channelLockHandler;
    PolarExtentPanner _polarExtentPanner;
    ZoneExclusionHandler _zoneExclusionHandler;
    Eigen::Array<bool, Eigen::Dynamic, 1> _isLfe;
  };

}  // namespace ear
