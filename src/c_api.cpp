#include "ear/c_api.h"
#include "ear/bs2051.hpp"
#include "ear/decorrelate.hpp"
#include "ear/layout.hpp"
#include "ear/metadata.hpp"
#include "ear/warnings.hpp"
#include "object_based/gain_calculator_objects.hpp"

#ifdef __wasm__
#include "wasm_api.hpp"
#define DEFAULT_WARNING_CB ::ear::wasm_warning_cb
#else
#define DEFAULT_WARNING_CB ::ear::default_warning_cb
#endif

namespace {
  /// delete *ptr and set to null if not already null
  template <typename T>
  void do_free(T **ptr) {
    assert(ptr != NULL);
    if (*ptr) {
      delete *ptr;
      *ptr = NULL;
    }
  }

  // implementation of OutputGains for float /double pointers
  template <typename T>
  class OutputGainsPtr : public ear::OutputGains {
   private:
    T *ptr;
    size_t size_;

   public:
    OutputGainsPtr(T *ptr, size_t size_) : ptr(ptr), size_(size_) {}
    virtual void check_size(size_t n) override {
      if (size_ != n) {
        ear_throw(ear::invalid_argument("incorrect size for output vector"));
      }
    }
    virtual size_t size() override { return size_; }
    virtual void write(size_t i, double x) override { ptr[i] = (T)x; }
    virtual void zero() override { std::fill(ptr, ptr + size_, (T)0.0); }
  };
}  // namespace

namespace ear {
  void wasm_warning_cb(const Warning &warning);
}

extern "C" {

//////////////////////
// ObjectsTypeMetadata
//////////////////////

struct ear_objects_type_metadata : ear::ObjectsTypeMetadata {};

ear_objects_type_metadata *ear_objects_type_metadata_new() {
  return new ear_objects_type_metadata{};
}

void ear_objects_type_metadata_free(ear_objects_type_metadata **ptr) {
  do_free(ptr);
}

void ear_objects_type_metadata_reset(ear_objects_type_metadata *ptr) {
  *ptr = {};
}

void ear_objects_type_metadata_set_polar_position(
    ear_objects_type_metadata *ptr, double azimuth, double elevation,
    double distance) {
  ptr->position = ear::PolarPosition{azimuth, elevation, distance};
}
void ear_objects_type_metadata_set_extent(ear_objects_type_metadata *ptr,
                                          double width, double height,
                                          double depth) {
  ptr->width = width;
  ptr->height = height;
  ptr->depth = depth;
}
void ear_objects_type_metadata_set_gain(ear_objects_type_metadata *ptr,
                                        double gain) {
  ptr->gain = gain;
}
void ear_objects_type_metadata_set_diffuse(ear_objects_type_metadata *ptr,
                                           double diffuse) {
  ptr->diffuse = diffuse;
}

/////////
// Layout
/////////

struct ear_layout : ear::Layout {
  using ear::Layout::Layout;
  ear_layout(ear::Layout &&layout) : ear::Layout(std::move(layout)) {}
  ear_layout(const ear::Layout &layout) : ear::Layout(layout) {}
};

ear_layout *ear_layout_get(const char *name) {
  auto &layouts = ear::loadLayouts();
  for (auto &layout : layouts)
    if (layout.name() == name) return new ear_layout{layout};
  return NULL;
}

void ear_layout_free(ear_layout **ptr) { do_free(ptr); }

size_t ear_layout_num_channels(ear_layout *ptr) {
  return ptr->channels().size();
}

////////////////////////
// GainCalculatorObjects
////////////////////////

// use Impl rather than C++ API to avoid another layer of PIMPL and vector
// copying
struct ear_gain_calculator_objects : ear::GainCalculatorObjectsImpl {
  using ear::GainCalculatorObjectsImpl::GainCalculatorObjectsImpl;
};

ear_gain_calculator_objects *ear_gain_calculator_objects_new(
    ear_layout *layout) {
  return new ear_gain_calculator_objects{*layout};
}

void ear_gain_calculator_objects_free(ear_gain_calculator_objects **ptr) {
  do_free(ptr);
}

static void ear_gain_calculator_objects_calc_gains_impl(
    ear_gain_calculator_objects *ptr, ear_objects_type_metadata *otm,
    size_t n_gains, double *direct_gains, double *diffuse_gains,
    const ear::WarningCB &warning_cb) {
  OutputGainsPtr<double> direct_wrap(direct_gains, n_gains);
  OutputGainsPtr<double> diffuse_wrap(diffuse_gains, n_gains);

  ptr->calculate(*otm, direct_wrap, diffuse_wrap, warning_cb);
}

void ear_gain_calculator_objects_calc_gains_cb(ear_gain_calculator_objects *ptr,
                                               ear_objects_type_metadata *otm,
                                               size_t n_gains,
                                               double *direct_gains,
                                               double *diffuse_gains,
                                               ear_warning_cb warning_cb) {
  ear_gain_calculator_objects_calc_gains_impl(
      ptr, otm, n_gains, direct_gains, diffuse_gains,
      [&](auto warning) { warning_cb(warning.message.c_str()); });
}

void ear_gain_calculator_objects_calc_gains(ear_gain_calculator_objects *ptr,
                                            ear_objects_type_metadata *otm,
                                            size_t n_gains,
                                            double *direct_gains,
                                            double *diffuse_gains) {
  ear_gain_calculator_objects_calc_gains_impl(
      ptr, otm, n_gains, direct_gains, diffuse_gains, DEFAULT_WARNING_CB);
}

////////////////
// decorrelators
////////////////

int ear_decorrelator_compensation_delay() {
  return ear::decorrelatorCompensationDelay();
}

float *ear_design_decorrelator(ear_layout *layout, size_t channel_idx,
                               size_t *length) {
  assert(length != NULL);

  auto dec_vector = ear::designDecorrelator<double>(*layout, channel_idx);

  *length = dec_vector.size();

  float *dec_ptr = (float *)malloc(sizeof(*dec_ptr) * dec_vector.size());
  for (size_t s = 0; s < dec_vector.size(); s++) dec_ptr[s] = dec_vector[s];

  return dec_ptr;
}
}
