#include <stddef.h>
#include "export.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ear_warning_cb)(const char *warning);

//////////////////////
// ObjectsTypeMetadata
//////////////////////

typedef struct ear_objects_type_metadata ear_objects_type_metadata;

EAR_EXPORT
ear_objects_type_metadata *ear_objects_type_metadata_new();
EAR_EXPORT
void ear_objects_type_metadata_free(ear_objects_type_metadata **);
EAR_EXPORT
void ear_objects_type_metadata_reset(ear_objects_type_metadata *);

EAR_EXPORT
void ear_objects_type_metadata_set_polar_position(ear_objects_type_metadata *,
                                                  double azimuth,
                                                  double elevation,
                                                  double distance);
EAR_EXPORT
void ear_objects_type_metadata_set_extent(ear_objects_type_metadata *,
                                          double width, double height,
                                          double depth);
EAR_EXPORT
void ear_objects_type_metadata_set_gain(ear_objects_type_metadata *,
                                        double gain);
EAR_EXPORT
void ear_objects_type_metadata_set_diffuse(ear_objects_type_metadata *,
                                           double diffuse);

/////////
// Layout
/////////

typedef struct ear_layout ear_layout;

/// get a layout by its BS.2051 name (e.g. 4+5+0); returns null if there is no
/// layout with the given name
EAR_EXPORT
ear_layout *ear_layout_get(const char *name);
EAR_EXPORT
void ear_layout_free(ear_layout **);

EAR_EXPORT
size_t ear_layout_num_channels(ear_layout *);

////////////////////////
// GainCalculatorObjects
////////////////////////

typedef struct ear_gain_calculator_objects ear_gain_calculator_objects;

EAR_EXPORT
ear_gain_calculator_objects *ear_gain_calculator_objects_new(ear_layout *);
EAR_EXPORT
void ear_gain_calculator_objects_free(ear_gain_calculator_objects **);
EAR_EXPORT
void ear_gain_calculator_objects_calc_gains(ear_gain_calculator_objects *,
                                            ear_objects_type_metadata *,
                                            size_t n_gains,
                                            double *direct_gains,
                                            double *diffuse_gains);
#ifndef __wasm__
EAR_EXPORT
void ear_gain_calculator_objects_calc_gains_cb(
    ear_gain_calculator_objects *, ear_objects_type_metadata *, size_t n_gains,
    double *direct_gains, double *diffuse_gains, ear_warning_cb warning_cb);
#endif

////////////////
// decorrelators
////////////////

EAR_EXPORT int ear_decorrelator_compensation_delay();

/// design a decorrelation filter for channel_idx in layout
///
/// the number of samples will be written to *length; the result should be
/// freed using free()
EAR_EXPORT float *ear_design_decorrelator(ear_layout *layout,
                                          size_t channel_idx, size_t *length);

#ifdef __cplusplus
}
#endif
