# Neutral accepted-row value/CSV authority, usable without binary record I/O,
# full-shell Context, native state, or CUDA.
if(NOT TARGET robo_dyna_artifact_io)
  include("${CMAKE_CURRENT_LIST_DIR}/../../ArtifactIO.cmake")
endif()
add_library(robo_dyna_interval_values STATIC ../IntervalValues.cpp ../IntervalChunkPlan.cpp)
target_link_libraries(robo_dyna_interval_values PUBLIC robo_dyna_artifact_io)
target_compile_features(robo_dyna_interval_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_interval_values PRIVATE -fno-fast-math -ffp-contract=off)
