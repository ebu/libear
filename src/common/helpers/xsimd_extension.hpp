#pragma once
#include <xsimd/types/xsimd_all_registers.hpp>

namespace xsimd {
  // aliases for things which will be used via preprocessor defines, to avoid
  // quoting issues
  using avx2_fma = fma3<avx2>;

  // a version of generic which works with xsimd::dispatch
  struct generic_for_dispatch : generic {
    static constexpr bool supported() noexcept { return true; }
    static constexpr bool available() noexcept { return true; }
    static constexpr bool requires_alignment() noexcept { return false; }
    static constexpr unsigned version() noexcept {
      return generic::version(0, 0, 0);
    }
    static constexpr std::size_t alignment() noexcept { return 0; }
    static constexpr char const* name() noexcept {
      return "generic_for_dispatch";
    }
  };
}  // namespace xsimd
