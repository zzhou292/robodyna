include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_scope/PhysicalScope.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../source_assembly/SourceAssemblySpotweldInput.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/assembly/NodalNodeDomain.cmake")
add_library(robo_dyna_vehicle_type25_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehicleType25Source.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Mapping.cpp")
target_link_libraries(robo_dyna_vehicle_type25_source PUBLIC robo_dyna_physical_scope
  robo_dyna_source_assembly_spotweld_input tl_nodal_node_domain)
target_compile_features(robo_dyna_vehicle_type25_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_type25_source PRIVATE -fno-fast-math -ffp-contract=off)
