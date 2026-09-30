include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/WallTessellationLibrary.cmake")
add_library(robo_dyna_placed_canonical_wall STATIC
  "${CMAKE_CURRENT_LIST_DIR}/PlacedCanonicalWall.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PlacedCanonicalWallArtifacts.cpp")
target_link_libraries(robo_dyna_placed_canonical_wall PUBLIC robo_dyna_wall_tessellation PRIVATE robo_dyna_artifact_io)
target_compile_options(robo_dyna_placed_canonical_wall PRIVATE -fno-fast-math -ffp-contract=off)
