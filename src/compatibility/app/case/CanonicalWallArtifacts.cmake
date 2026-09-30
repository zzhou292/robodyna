# Shared case/replay authentication and archive of the original mesh wall.
include("${CMAKE_CURRENT_LIST_DIR}/CanonicalWall.cmake")
if(NOT TARGET robo_dyna_canonical_wall_artifacts)
  add_library(robo_dyna_canonical_wall_artifacts STATIC "${CMAKE_CURRENT_LIST_DIR}/CanonicalWallArtifacts.cpp")
  target_link_libraries(robo_dyna_canonical_wall_artifacts PUBLIC crash_canonical_wall PRIVATE robo_dyna_artifact_io)
  target_compile_options(robo_dyna_canonical_wall_artifacts PRIVATE -fno-fast-math -ffp-contract=off)
endif()
