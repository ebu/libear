#pragma once
#include <stdexcept>
#include <string>
#include "export.hpp"

namespace ear {

/// support for building without exceptions
///
/// in the rest of the code, the ear_throw macro is used to throw exceptions,
/// and normally this just throws the given exception
///
/// when exceptions are disabled, it instead calls ear::handle_exception with
/// the exception object, and can be provided by users of the library
///
/// this callback must return out of the library (e.g. by abort or longjmp),
/// and you can't assume that the library is in a reasonable state after this
/// has happened, because clean-up that would normally be done during unwinding
/// will not happen
///
/// this is really unfortunate, but the library was designed with use of C++
/// exceptions in mind, so a lot of code would need to change to fix this
///
/// note that boost has a similar system which requires defining
/// boost::throw_exception
///
/// an example of both is provided in src/handle_exceptions.cpp, which is used
/// to build examples and tests
#ifdef EAR_NO_EXCEPTIONS

  [[noreturn]] void handle_exception(const std::exception &e);

#define ear_throw(exc) ::ear::handle_exception(exc);

#else
#define ear_throw(exc) throw exc
#endif

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
