#include <boost/make_unique.hpp>
#include "../common/helpers/xsimd_extension.hpp"
#include "polar_extent_simd.hpp"

namespace ear {

  template <typename arch>
  std::unique_ptr<PolarExtentCore> get_polar_extent_core_arch() {
    return boost::make_unique<PolarExtentCoreSimd<arch>>();
  }

  template std::unique_ptr<PolarExtentCore>
  get_polar_extent_core_arch<XSIMD_ARCH>();
}  // namespace ear
