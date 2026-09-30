include_guard(GLOBAL)
if(NOT TARGET robo_dyna_full_shell_source_mapping)
  set(ROBO_DYNA_FULL_SHELL_RECORD_TESTS OFF CACHE BOOL "Use owning record gate")
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../full_shell" "${CMAKE_CURRENT_BINARY_DIR}/physical-frame-records")
endif()
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchLayeredSection.h")
  message(FATAL_ERROR "Physical accepted fields require the explicit qualified TL-FEA root")
endif()
find_package(CUDAToolkit REQUIRED)
find_package(Eigen3 REQUIRED)
add_library(robo_dyna_physical_frame_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Buffers.cpp" "${CMAKE_CURRENT_LIST_DIR}/Fields.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MappingRoles.cpp" "${CMAKE_CURRENT_LIST_DIR}/Phase.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/EnvironmentFields.cpp")
include("${CMAKE_CURRENT_LIST_DIR}/FrameArchive.cmake")
target_link_libraries(robo_dyna_physical_frame_values PUBLIC robo_dyna_physical_frame_archive Eigen3::Eigen CUDA::cudart)
target_include_directories(robo_dyna_physical_frame_values PUBLIC "${ROBO_DYNA_TL_ROOT}" "${ROBO_DYNA_TL_ROOT}/lib_src")
target_compile_features(robo_dyna_physical_frame_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_physical_frame_values PRIVATE -fno-fast-math -ffp-contract=off)
option(ROBO_DYNA_PHYSICAL_CAPTURE_RUNTIME "Build actual accepted physical-owner capture" OFF)
if(ROBO_DYNA_PHYSICAL_CAPTURE_RUNTIME)
  include("${CMAKE_CURRENT_LIST_DIR}/../../case/vehicle_runtime/VehiclePhysicalStartup.cmake")
  add_library(robo_dyna_physical_accepted_frames STATIC
    "${CMAKE_CURRENT_LIST_DIR}/Mapping.cpp" "${CMAKE_CURRENT_LIST_DIR}/PhaseIdentity.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/CaptureContext.cpp" "${CMAKE_CURRENT_LIST_DIR}/Capture.cpp")
  target_link_libraries(robo_dyna_physical_accepted_frames PUBLIC robo_dyna_physical_frame_values
    robo_dyna_vehicle_physical_startup robo_dyna_full_shell_source_bundle)
  target_compile_features(robo_dyna_physical_accepted_frames PUBLIC cxx_std_17)
  target_compile_options(robo_dyna_physical_accepted_frames PRIVATE -fno-fast-math -ffp-contract=off)
endif()
