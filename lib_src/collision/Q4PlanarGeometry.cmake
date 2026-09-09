# Reusable host preparation; no CUDA state or runtime dependency.
if(NOT TARGET tl_q4_planar_geometry)
  include("${CMAKE_CURRENT_LIST_DIR}/PlanarWallGeometry.cmake")
  add_library(tl_q4_planar_geometry STATIC "${CMAKE_CURRENT_LIST_DIR}/Q4PlanarGeometry.cpp")
  target_compile_features(tl_q4_planar_geometry PUBLIC cxx_std_17)
  target_compile_options(tl_q4_planar_geometry PRIVATE -fno-fast-math -ffp-contract=off)
  target_link_libraries(tl_q4_planar_geometry PUBLIC tl_planar_wall_geometry)
endif()
