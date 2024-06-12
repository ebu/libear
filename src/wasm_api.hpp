#pragma once
#include "ear/warnings.hpp"

#define WASM_EXPORT(f) __attribute__((export_name(#f))) f
#define WASM_IMPORT(f) __attribute__((import_name(#f))) f

extern "C" {

/// call to initialise the library (calls static constructors)
void WASM_EXPORT(ear_init)();

/// implement in JS to handle exceptions; must call abort or throw an exception
[[noreturn]] extern void WASM_IMPORT(ear_handle_exception)(const char *);
/// implement in JS to handle warnings
extern void WASM_IMPORT(ear_handle_warning)(const char *);
}

namespace ear {
  /// c++ warning callback which calls ear_handle_warning
  ///
  /// this is used because passing function pointers in wasm is tricky, and not
  /// really necessary as calls are always in the same thread
  void wasm_warning_cb(const ear::Warning &warning);
}  // namespace ear
