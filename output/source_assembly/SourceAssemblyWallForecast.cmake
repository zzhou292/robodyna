# Shared archive arithmetic only: no source startup, mechanics or CUDA targets.
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../ArtifactIO.cmake")
add_library(robo_dyna_source_assembly_wall_forecast STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallForecast.cpp")
target_link_libraries(robo_dyna_source_assembly_wall_forecast PUBLIC robo_dyna_artifact_io)
target_compile_features(robo_dyna_source_assembly_wall_forecast PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_wall_forecast PRIVATE -fno-fast-math -ffp-contract=off)
