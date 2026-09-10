include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyBindings.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../shell_collection/ShellCollectionContactGeometry.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../wall_penalty/WallPenaltyCertification.cmake")
add_library(robo_dyna_source_assembly_initial_kinetic STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyInitialKinetic.cpp")
target_link_libraries(robo_dyna_source_assembly_initial_kinetic PUBLIC
  robo_dyna_source_assembly_bindings robo_dyna_wall_penalty_certification)
target_compile_options(robo_dyna_source_assembly_initial_kinetic PRIVATE -fno-fast-math -ffp-contract=off)
