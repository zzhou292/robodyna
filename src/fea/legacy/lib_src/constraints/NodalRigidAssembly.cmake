include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/NodalRigidGroup.cmake")
add_library(tl_nodal_rigid_assembly STATIC
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidAssemblyBody.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidAssemblyMerge.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidAssemblyFinalize.cpp")
target_link_libraries(tl_nodal_rigid_assembly PUBLIC tl_nodal_rigid_group)
target_compile_features(tl_nodal_rigid_assembly PUBLIC cxx_std_17)
target_compile_options(tl_nodal_rigid_assembly PRIVATE -fno-fast-math -ffp-contract=off)
