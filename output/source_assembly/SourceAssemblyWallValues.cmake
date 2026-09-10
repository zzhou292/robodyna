include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallForecast.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyOutput.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/Q4PlanarGeometry.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../case/CanonicalWallArtifacts.cmake")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
include("${CMAKE_CURRENT_LIST_DIR}/../../case/source_assembly/SourceAssemblyWallSetup.cmake")
# Pure host serialization/ordering. Headers describe the case's value fields;
# this target links no state owner, CUDA runtime step or material batch owner.
add_library(robo_dyna_source_assembly_wall_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallSequence.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallStamp.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallFrameChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallFrameFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallDiagnostics.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallForceStageFields.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallContactFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallInputFields.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallSetupFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallConfiguration.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallInterval.cpp" "${CMAKE_CURRENT_LIST_DIR}/WallArtifactFileIO.cpp")
target_link_libraries(robo_dyna_source_assembly_wall_values PUBLIC
  robo_dyna_source_assembly_fields robo_dyna_source_assembly_wall_setup robo_dyna_source_assembly_wall_forecast robo_dyna_artifact_io)
target_compile_features(robo_dyna_source_assembly_wall_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_wall_values PRIVATE -fno-fast-math -ffp-contract=off)
