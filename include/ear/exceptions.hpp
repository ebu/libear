#pragma once
#include <stdexcept>
#include "export.hpp"

namespace ear {
  /// thrown if features are used which are not yet implemented
  class EAR_EXPORT not_implemented : public std::runtime_error {
   public:
    explicit not_implemented(const std::string &what)
        : std::runtime_error("not implemented: " + what) {}
  };

  /// thrown for errors inside the library, which should not have occurred given
  /// any inputs. This can be caused by an error in the library itself (please
  /// report it!) or something going wrong while building the library. This is
  /// thrown by ear_assert.
  class EAR_EXPORT internal_error : public std::runtime_error {
   public:
    explicit internal_error(const std::string &what)
        : std::runtime_error("internal error: " + what) {}
  };

  /// thrown if invalid ADM metadata is encountered
  class EAR_EXPORT adm_error : public std::invalid_argument {
   public:
    explicit adm_error(const std::string &what)
        : std::invalid_argument("ADM error: " + what) {}
  };

  /// thrown if an unknown loudspeaker layout is requested
  class EAR_EXPORT unknown_layout : public std::invalid_argument {
   public:
    explicit unknown_layout(const std::string &what)
        : std::invalid_argument("unknown layout: " + what) {}
  };

  /// thrown if other invariants on parameters are not met
  class EAR_EXPORT invalid_argument : public std::invalid_argument {
   public:
    explicit invalid_argument(const std::string &what)
        : std::invalid_argument(what) {}
  };

}  // namespace ear
