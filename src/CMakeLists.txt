# --- ear library ---

include(embed_resource)
embed_resource(
  NAMESPACE ear
  BASE_NAME resources
  RESOURCE_FILES "${PROJECT_SOURCE_DIR}/resources/Design_5200_100_random.dat")

add_library(ear
  bs2051.cpp
  bs2051_layouts.cpp
  common/convex_hull.cpp
  common/facets.cpp
  common/geom.cpp
  common/point_source_panner.cpp
  decorrelate.cpp
  direct_speakers/gain_calculator_direct_speakers.cpp
  direct_speakers/mapping_rules.cpp
  dsp/block_convolver.cpp
  dsp/block_convolver_impl.cpp
  dsp/delay_buffer_impl.cpp
  dsp/variable_block_size_impl.cpp
  gain_calculators.cpp
  fft_kiss.cpp
  hoa/gain_calculator_hoa.cpp
  hoa/hoa.cpp
  layout.cpp
  screen.cpp
  object_based/extent.cpp
  object_based/gain_calculator_objects.cpp
  warnings.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/resources.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/resources.cpp
)

# only include HTM-related sources if not disabled; these are used in tests, so
# must be absolute paths
set(EAR_HTM_SOURCES
    ${PROJECT_SOURCE_DIR}/src/object_based/extent_htm.cpp
    ${PROJECT_SOURCE_DIR}/src/object_based/htm.cpp
)
set(EAR_HTM_SOURCES "${EAR_HTM_SOURCES}" PARENT_SCOPE)
if(EAR_DISABLE_HTM)
    target_compile_definitions(ear PRIVATE EAR_DISABLE_HTM)
else()
    target_sources(ear PRIVATE ${EAR_HTM_SOURCES})
endif()

target_include_directories(ear
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/submodules/kissfft>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # resources.hpp
  PUBLIC
  # Headers used from source/build location:
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>  # config.h / export.h
  # Headers used from installed location:
    $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
)

target_link_libraries(ear PUBLIC Boost::boost ${CMAKE_DL_LIBS})
# Only use include directory of Eigen3 target
# as otherwise Eigen3 needs to be installed and found when linking to a
# static version of libear, even though it's an INTERFACE target with PRIVATE linking
# This is because cmake doesn't check if the INTERFACE target itself might have linker dependencies
# or not (see https://gitlab.kitware.com/cmake/cmake/issues/18049)
if(EAR_HIDE_INTERNAL_SYMBOLS)
    target_include_directories(ear PRIVATE $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>)
else()
    target_include_directories(ear PUBLIC $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>)
endif()

set_property(TARGET ear PROPERTY POSITION_INDEPENDENT_CODE ON)

if(EAR_HIDE_INTERNAL_SYMBOLS AND BUILD_SHARED_LIBS)
  set_target_properties(ear PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    C_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN TRUE)
endif()

############################################################
# enable C++11 support
############################################################
if (${CMAKE_VERSION} VERSION_LESS "3.8.0")
  # Note: this is not a complete list of c++ features required by libear.
  # What we want is C++11 support and this is a simple way to trigger
  # this for CMake < 3.8
  target_compile_features(ear PUBLIC cxx_auto_type cxx_nullptr cxx_range_for)
else()
  target_compile_features(ear PUBLIC cxx_std_11)
endif()
set_target_properties(ear PROPERTIES CXX_EXTENSIONS OFF)

include(GenerateExportHeader)
generate_export_header(ear
  EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/generated/export.hpp
)

############################################################
# export package
############################################################
if(EAR_PACKAGE_AND_INSTALL)
  include(CMakePackageConfigHelpers)

  configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/config/earConfig.cmake.in
    ${PROJECT_BINARY_DIR}/earConfig.cmake
    INSTALL_DESTINATION
    ${INSTALL_CMAKE_DIR}
    PATH_VARS INSTALL_INCLUDE_DIR INSTALL_LIB_DIR INSTALL_CMAKE_DIR
  )

  write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/earConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
  )

  ############################################################
  # install ear library
  ############################################################
  set_target_properties(ear PROPERTIES
    VERSION "${PROJECT_VERSION}"
    SOVERSION "${PROJECT_VERSION_MAJOR}"
  )

  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ear/
    DESTINATION ${INSTALL_INCLUDE_DIR}/ear
  )
  install(DIRECTORY ${PROJECT_BINARY_DIR}/generated/
    DESTINATION ${INSTALL_INCLUDE_DIR}/ear/generated
  )

  install(FILES
    ${PROJECT_BINARY_DIR}/earConfig.cmake
    ${PROJECT_BINARY_DIR}/earConfigVersion.cmake
    DESTINATION ${INSTALL_CMAKE_DIR}
  )

  if(${BUILD_SHARED_LIBS})
    install(TARGETS ear
      EXPORT earTargetsShared
      LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
      RUNTIME DESTINATION "${INSTALL_LIB_DIR}"
      ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
      PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
    )
    install(EXPORT earTargetsShared DESTINATION ${INSTALL_CMAKE_DIR})
  else()
    install(TARGETS ear
      EXPORT earTargetsStatic
      LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
      RUNTIME DESTINATION "${INSTALL_LIB_DIR}"
      ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
      PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
    )
    install(EXPORT earTargetsStatic DESTINATION ${INSTALL_CMAKE_DIR})
  endif()
endif()
