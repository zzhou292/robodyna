include_guard(GLOBAL)
add_library(tl_planar_wall_geometry STATIC
  "${CMAKE_CURRENT_LIST_DIR}/PlanarWallGeometry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PlanarWallBox.cpp")
get_filename_component(TL_PLANAR_GEOMETRY_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(tl_planar_wall_geometry PUBLIC "${TL_PLANAR_GEOMETRY_ROOT}")
target_compile_features(tl_planar_wall_geometry PUBLIC cxx_std_17)
target_compile_options(tl_planar_wall_geometry PRIVATE -fno-fast-math -ffp-contract=off)
