#include <xsimd/xsimd.hpp>
#include "../common/helpers/xsimd_extension.hpp"
#include "polar_extent.hpp"

namespace ear {
  using arch_list = xsimd::arch_list<XSIMD_ARCHS>;

  struct GetPolarExtentCore {
    template <typename Arch>
    std::unique_ptr<PolarExtentCore> operator()(Arch) {
      return get_polar_extent_core_arch<Arch>();
    }

    std::unique_ptr<PolarExtentCore> operator()(xsimd::generic_for_dispatch) {
      return get_polar_extent_core_scalar();
    }
  };

  std::unique_ptr<PolarExtentCore> get_polar_extent_core() {
    auto dispatcher = xsimd::dispatch<arch_list>(GetPolarExtentCore{});
    return dispatcher();
  }
}  // namespace ear
