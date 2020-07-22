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
  conversion.cpp
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
  object_based/polar_extent.cpp
  object_based/polar_extent_scalar.cpp
  object_based/polar_extent_simd.cpp
  object_based/gain_calculator_objects.cpp
  warnings.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/resources.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/resources.cpp
)

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
target_include_directories(ear PRIVATE $<TARGET_PROPERTY:xsimd,INTERFACE_INCLUDE_DIRECTORIES>)
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
# enable C++14 support
############################################################
if (${CMAKE_VERSION} VERSION_LESS "3.8.0")
  # Note: this is not a complete list of c++ features required by libadm.
  # What we want is C++14 support and this is a simple way to trigger
  # this for CMake < 3.8
  target_compile_features(ear PUBLIC cxx_generic_lambdas)
else()
  target_compile_features(ear PUBLIC cxx_std_14)
endif()
set_target_properties(ear PROPERTIES CXX_EXTENSIONS OFF)

include(CheckCompilerFlag)

set(PER_ARCH_SOURCES object_based/polar_extent_simd_instance.cpp)
function(add_arch_lib TARGET ARCH FLAGS)
    check_compiler_flag(CXX "${FLAGS}" ear_has_flag_${ARCH}) # var is cached and must be unique
    if (ear_has_flag_${ARCH})
        list(APPEND XSIMD_ARCHS xsimd::${ARCH})
        add_library(${TARGET}_${ARCH} OBJECT ${PER_ARCH_SOURCES})
        target_sources(${TARGET} PRIVATE $<TARGET_OBJECTS:${TARGET}_${ARCH}>)

        # copy relevant target properties to feature-specific objects
        foreach(PROPERTY COMPILE_DEFINITIONS COMPILE_FEATURES INCLUDE_DIRECTORIES LINK_LIBRARIES
                POSITION_INDEPENDENT_CODE CXX_VISIBILITY_PRESET C_VISIBILITY_PRESET VISIBILITY_INLINES_HIDDEN)
            get_target_property(VALUE ${TARGET} ${PROPERTY})
            if (NOT VALUE STREQUAL "VALUE-NOTFOUND")
                set_property(TARGET ${TARGET}_${ARCH} PROPERTY ${PROPERTY} ${VALUE})
            endif()
        endforeach()

        target_compile_definitions(${TARGET}_${ARCH} PRIVATE XSIMD_ARCH=xsimd::${ARCH})

        target_compile_options(${TARGET}_${ARCH} PRIVATE ${FLAGS})
    endif()

    set(XSIMD_ARCHS "${XSIMD_ARCHS}" PARENT_SCOPE)
endfunction()

if (EAR_SIMD)
    # add_arch_lib must be called from best to worst instruction set
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|X86|i686")
        if (MSVC)
            add_arch_lib(ear avx512bw "/arch:AVX512")
            add_arch_lib(ear avx2_fma "/arch:AVX2")
            add_arch_lib(ear avx "/arch:AVX")
        else()
            add_arch_lib(ear avx512bw "-mavx512bw;-mavx512dq;-mavx512cd;-mavx512f;-mfma;-mavx2;-mavx")
            add_arch_lib(ear avx2_fma "-mfma;-mavx2;-mavx")
            add_arch_lib(ear avx "-mavx")
            add_arch_lib(ear sse4_2 "-msse4.2")
        endif()
        add_arch_lib(ear default_arch "")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "ARM.*|arm.*|aarch64.*")
        # for arm, flags are not generally needed as the FP extension support is
        # closely tied to the ISA, so just build the default and fallback
        #
        # this can go wrong if the compiler doesn't actually support neon, but it's
        # tricky to detect that
        add_arch_lib(ear default_arch "")
    endif()
endif()

list(APPEND XSIMD_ARCHS xsimd::generic_for_dispatch)

list(JOIN XSIMD_ARCHS "," XSIMD_ARCHS_STR)
target_compile_definitions(ear PRIVATE "XSIMD_ARCHS=${XSIMD_ARCHS_STR}")

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
      RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
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
