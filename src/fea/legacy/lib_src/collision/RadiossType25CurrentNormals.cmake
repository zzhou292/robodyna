include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25FixedMainStartup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25NormalActivation.cmake")
add_library(tl_radioss_type25_current_normals STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/current_normals/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/current_normals/Build.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/current_normals/MixedSource.cpp")
target_link_libraries(tl_radioss_type25_current_normals PUBLIC
  tl_radioss_type25_fixed_main_startup tl_radioss_type25_normal_activation)
# Production C++/CUDA arithmetic only; native Fortran is a qualification target.
