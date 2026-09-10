include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/QephSpinFields.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallArtifacts.cmake")
add_library(robo_dyna_source_assembly_spin_trace STATIC "${CMAKE_CURRENT_LIST_DIR}/QephSpinTrace.cpp")
target_link_libraries(robo_dyna_source_assembly_spin_trace PUBLIC
  robo_dyna_source_assembly_spin_fields robo_dyna_source_assembly_wall_artifacts)
target_compile_features(robo_dyna_source_assembly_spin_trace PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_spin_trace PRIVATE -fno-fast-math -ffp-contract=off)
