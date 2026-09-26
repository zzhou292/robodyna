include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25SurfaceSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25FixedMainStartup.cmake")
add_library(tl_radioss_type25_interface_surface STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/surface_interface/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/surface_interface/Prepare.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/surface_interface/Classify.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/surface_interface/Filter.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/surface_interface/Build.cpp")
target_link_libraries(tl_radioss_type25_interface_surface PUBLIC
  tl_radioss_type25_surface_source tl_radioss_type25_fixed_main_startup)
target_compile_features(tl_radioss_type25_interface_surface PUBLIC cxx_std_17)
target_compile_options(tl_radioss_type25_interface_surface PRIVATE -fno-fast-math -ffp-contract=off)
