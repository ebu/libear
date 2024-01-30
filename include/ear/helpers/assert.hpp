#pragma once
#include "../exceptions.hpp"

namespace ear {
  // implementation for _assert_impl. This is wrapped in a macro so that we can
  // use __LINE__, __FILE__ etc. in the future.
  inline void _assert_impl(bool condition, const char *message) {
    if (!condition) ear_throw(internal_error(message));
  }

  inline void _assert_impl(bool condition, const std::string &message) {
    if (!condition) ear_throw(internal_error(message));
  }
}  // namespace ear

// assert which is always enabled, and results in an ear::internal_error with
// the given message
#define ear_assert(condition, message) ear::_assert_impl((condition), (message))
