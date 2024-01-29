# determine export flags to pass when linking a wasm module

# we would like this to be as small as possible, but not all functions are
# needed all the time, so groups of exports are defined which can be set with
# EAR_WASM_EXPORT_GROUPS

# wasm_add_export adds a function name the wasm_exports_[group] lists, which are
# then combined into one list (wasm_exports) and ultimately a list of flags
# (wasm_export_flags)

# it would be possible to put this in the source instead, but would require some
# tricky macros

macro(wasm_add_export group name)
  list(APPEND wasm_exports_${group} ${name})
  list(APPEND wasm_exports_all ${name})

  # append to wasm_export_groups if not present
  list(FIND wasm_export_groups ${group} wasm_has_group)
  if(wasm_has_group EQUAL -1)
    list(APPEND wasm_export_groups ${group})
  endif()
endmacro()

# for all functions in c_api.h, add a call here with a group to put it in

wasm_add_export(objects ear_objects_type_metadata_new)
wasm_add_export(objects ear_objects_type_metadata_free)
wasm_add_export(objects ear_objects_type_metadata_reset)
wasm_add_export(objects ear_objects_type_metadata_set_polar_position)
wasm_add_export(objects ear_objects_type_metadata_set_extent)
wasm_add_export(objects ear_objects_type_metadata_set_gain)
wasm_add_export(objects ear_objects_type_metadata_set_diffuse)

wasm_add_export(bs2051 ear_layout_get)
wasm_add_export(layout ear_layout_num_channels)
wasm_add_export(layout ear_layout_free)

wasm_add_export(objects ear_gain_calculator_objects_new)
wasm_add_export(objects ear_gain_calculator_objects_free)
wasm_add_export(objects ear_gain_calculator_objects_calc_gains)

wasm_add_export(decorrelators ear_decorrelator_compensation_delay)
wasm_add_export(decorrelators ear_design_decorrelator)

set(EAR_WASM_EXPORT_GROUPS
    "all"
    CACHE
      STRING
      "groups of wasm functions to export; options: ${wasm_export_groups} or all"
)

# always required
set(wasm_exports malloc free ear_init)

foreach(group IN LISTS EAR_WASM_EXPORT_GROUPS)
  foreach(export IN LISTS wasm_exports_${group})
    # append to wasm_exports if not present
    list(FIND wasm_exports ${export} wasm_has_export)
    if(wasm_has_export EQUAL -1)
      list(APPEND wasm_exports ${export})
    endif()
  endforeach()
endforeach()

list(TRANSFORM wasm_exports PREPEND "-Wl,--export=" OUTPUT_VARIABLE
                                                    wasm_export_flags)
