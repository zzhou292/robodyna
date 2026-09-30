include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/NodalRigidAssembly.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/NodalRigidPartTopology.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../assembly/NodalCoefficientLedger.cmake")
add_library(tl_nodal_rigid_part_assembly_model STATIC
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidPartAssemblyModel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidPartAssemblyBudget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidPartAssemblyTopology.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidPartAssemblyMap.cpp")
target_link_libraries(tl_nodal_rigid_part_assembly_model PUBLIC
  tl_nodal_rigid_assembly tl_nodal_rigid_part_topology tl_nodal_coefficient_ledger)
target_compile_features(tl_nodal_rigid_part_assembly_model PUBLIC cxx_std_17)
target_compile_options(tl_nodal_rigid_part_assembly_model PRIVATE -fno-fast-math -ffp-contract=off)
