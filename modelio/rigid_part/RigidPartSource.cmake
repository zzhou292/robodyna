include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../tied_shell/TiedShellDeclaration.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/NodalRigidPartTopology.cmake")
add_library(robo_dyna_rigid_part_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/RigidPartDeclarations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RigidPartSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Declarations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Sources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Connections.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Geometry.cpp")
target_link_libraries(robo_dyna_rigid_part_source PUBLIC robo_dyna_tied_shell_declaration tl_nodal_rigid_part_topology)
target_compile_features(robo_dyna_rigid_part_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_rigid_part_source PRIVATE -fno-fast-math -ffp-contract=off)
