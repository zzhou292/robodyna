include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/NodalRigidPartAssemblyModel.cmake")
add_library(tl_nodal_rigid_assembly_binding STATIC
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidAssemblyBinding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidAssemblyBindingBudget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidAssemblyBindingMap.cpp")
target_link_libraries(tl_nodal_rigid_assembly_binding PUBLIC tl_nodal_rigid_part_assembly_model)
target_compile_features(tl_nodal_rigid_assembly_binding PUBLIC cxx_std_17)
target_compile_options(tl_nodal_rigid_assembly_binding PRIVATE -fno-fast-math -ffp-contract=off)
