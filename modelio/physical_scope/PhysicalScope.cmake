include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../beam18/Source.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../rigid_part/point_mass/Source.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../solid_source/VehicleSolidSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../type13/SourceType13.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/assembly/NodalNodeDomain.cmake")
add_library(robo_dyna_physical_scope STATIC
  "${CMAKE_CURRENT_LIST_DIR}/CanonicalDomain.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/DomainEmbedding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PhysicalScope.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Connections.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Groups.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Roles.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/BeamRoles.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Evidence.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Report.cpp")
target_link_libraries(robo_dyna_physical_scope PUBLIC robo_dyna_rigid_point_mass_source
  robo_dyna_vehicle_solid_source robo_dyna_type13_source robo_dyna_beam18_source tl_nodal_node_domain)
target_compile_features(robo_dyna_physical_scope PUBLIC cxx_std_17)
target_compile_options(robo_dyna_physical_scope PRIVATE -fno-fast-math -ffp-contract=off)
