include_guard(GLOBAL)
# Shared host wall preparation; callers opt into their own checks/cases.
add_library(robo_dyna_wall_tessellation STATIC
  "${CMAKE_CURRENT_LIST_DIR}/WallTessellation.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/WallTessellationGeometry.cpp")
target_include_directories(robo_dyna_wall_tessellation PUBLIC "${CRASH_TL_FEA_SOURCE_DIR}/lib_src")
target_link_libraries(robo_dyna_wall_tessellation PUBLIC crash_canonical_wall tl_q4_planar_geometry PRIVATE
  robo_dyna_canonical_wall_artifacts robo_dyna_artifact_io)
target_compile_options(robo_dyna_wall_tessellation PRIVATE -fno-fast-math -ffp-contract=off)
