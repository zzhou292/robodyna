include_guard(GLOBAL)
get_filename_component(tl_part_topology_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_library(tl_nodal_rigid_part_topology STATIC
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidPartTopology.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalRigidPartTopologyChecks.cpp")
target_include_directories(tl_nodal_rigid_part_topology PUBLIC "${tl_part_topology_root}")
target_compile_features(tl_nodal_rigid_part_topology PUBLIC cxx_std_17)
