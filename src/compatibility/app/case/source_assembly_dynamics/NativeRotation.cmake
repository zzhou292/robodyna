include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../source_assembly/SourceAssemblyWallSetup.cmake")
find_package(CUDAToolkit REQUIRED)
# Config value headers declare CUDA stream types; no runtime link.
# Host observation values and immutable source geometry; no nodal owner/runtime.
add_library(robo_dyna_source_native_rotation STATIC
  "${CMAKE_CURRENT_LIST_DIR}/NativeRotation.cpp" "${CMAKE_CURRENT_LIST_DIR}/NativeRotationStartup.cpp")
target_link_libraries(robo_dyna_source_native_rotation PUBLIC robo_dyna_source_assembly_wall_setup)
target_include_directories(robo_dyna_source_native_rotation PUBLIC "${CUDAToolkit_INCLUDE_DIRS}")
target_compile_features(robo_dyna_source_native_rotation PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_native_rotation PRIVATE -fno-fast-math -ffp-contract=off)
