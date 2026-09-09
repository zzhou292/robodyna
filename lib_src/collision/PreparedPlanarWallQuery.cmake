include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/PlanarWallGeometry.cmake")
add_library(tl_prepared_planar_wall_query STATIC
  "${CMAKE_CURRENT_LIST_DIR}/PreparedPlanarWallQuery.cpp")
target_link_libraries(tl_prepared_planar_wall_query PUBLIC tl_planar_wall_geometry)
target_compile_features(tl_prepared_planar_wall_query PUBLIC cxx_std_17)
target_compile_options(tl_prepared_planar_wall_query PRIVATE -fno-fast-math -ffp-contract=off)
