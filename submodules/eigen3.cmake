if(NOT EXISTS "${PROJECT_SOURCE_DIR}/submodules/eigen/Eigen/Eigen")
    message(SEND_ERROR
        "Eigen submodule has not been cloned; please run:\n"
        "git submodule update --init --recursive"
    )
endif()

add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
set_target_properties(Eigen3::Eigen PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/submodules/eigen>
)
