#include "wasm_api.hpp"
#include <boost/throw_exception.hpp>
#include <cstdlib>
#include <stdexcept>
#include "ear/exceptions.hpp"
#include "ear/warnings.hpp"

extern "C" {
// run static constructors once -- the compiler detects this and doesn't run
// them inside every function as usual
extern void __wasm_call_ctors();
void WASM_EXPORT(ear_init)() { __wasm_call_ctors(); }

// override things in the system libraries to avoid having to implement WASI
// imports just to print assertions which should never happen anyway
//
// this is bad and presumably depends on link order

// used in libcxxabi
[[noreturn]] void abort_message(const char *format, ...) {
  ear_handle_exception(format);
}

// used by assert in wasi-libc
[[noreturn]] void __assert_fail(const char *expr, const char *file, int line,
                                const char *func) {
  ear_handle_exception(expr);
}
}

namespace ear {

  [[noreturn]] void handle_exception(const std::exception &e) {
    ear_handle_exception(e.what());
  }

  void wasm_warning_cb(const ear::Warning &warning) {
    ear_handle_warning(warning.message.c_str());
  }
}  // namespace ear

#ifdef BOOST_NO_EXCEPTIONS
namespace boost {

  BOOST_NORETURN void throw_exception(std::exception const &e) { ear_throw(e); }

  BOOST_NORETURN void throw_exception(std::exception const &e,
                                      boost::source_location const &loc) {
    ear_throw(e);
  }

}  // namespace boost
#endif
