include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_scope/PhysicalScope.cmake")
add_library(robo_dyna_vehicle_physical_domain STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalDomain.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Selection.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ProfileSelection.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Topology.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp")
target_link_libraries(robo_dyna_vehicle_physical_domain PUBLIC robo_dyna_physical_scope tl_nodal_node_domain)
target_compile_features(robo_dyna_vehicle_physical_domain PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_physical_domain PRIVATE -fno-fast-math -ffp-contract=off)
