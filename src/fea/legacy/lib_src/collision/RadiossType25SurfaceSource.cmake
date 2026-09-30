include_guard(GLOBAL)
get_filename_component(TL_SURFACE_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_library(tl_radioss_type25_surface_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_surfaces/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_surfaces/Prepare.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_surfaces/Extract.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_surfaces/Build.cpp")
target_include_directories(tl_radioss_type25_surface_source PUBLIC "${TL_SURFACE_SOURCE_ROOT}")
target_compile_features(tl_radioss_type25_surface_source PUBLIC cxx_std_17)
target_compile_options(tl_radioss_type25_surface_source PRIVATE -fno-fast-math -ffp-contract=off)
