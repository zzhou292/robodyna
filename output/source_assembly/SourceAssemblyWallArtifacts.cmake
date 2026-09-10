include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallValues.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../case/source_assembly_dynamics/SourceAssemblyDynamics.cmake")
add_library(robo_dyna_source_assembly_wall_artifacts STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallArtifacts.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallArtifactFinish.cpp")
target_link_libraries(robo_dyna_source_assembly_wall_artifacts PUBLIC
  robo_dyna_source_assembly_wall_values robo_dyna_source_assembly_dynamics)
target_compile_features(robo_dyna_source_assembly_wall_artifacts PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_wall_artifacts PRIVATE -fno-fast-math -ffp-contract=off)
