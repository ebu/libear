#include <boost/throw_exception.hpp>
#include <cstdio>
#include <cstdlib>
#include "ear/exceptions.hpp"

#ifdef EAR_NO_EXCEPTIONS
namespace ear {

  [[noreturn]] void handle_exception(const std::exception &e) {
    fputs(e.what(), stderr);
    fputc('\n', stderr);
    std::abort();
  }

}  // namespace ear
#endif

#ifdef BOOST_NO_EXCEPTIONS
namespace boost {

  BOOST_NORETURN void throw_exception(std::exception const &e) { ear_throw(e); }

  BOOST_NORETURN void throw_exception(std::exception const &e,
                                      boost::source_location const &loc) {
    ear_throw(e);
  }

}  // namespace boost
#endif
