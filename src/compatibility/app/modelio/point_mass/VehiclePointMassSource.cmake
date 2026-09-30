include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_scope/PhysicalScope.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/assembly/ElementMassContributions.cmake")
add_library(robo_dyna_vehicle_point_mass_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehiclePointMassSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Mapping.cpp")
target_link_libraries(robo_dyna_vehicle_point_mass_source PUBLIC robo_dyna_physical_scope tl_element_mass_contributions)
target_compile_features(robo_dyna_vehicle_point_mass_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_point_mass_source PRIVATE -fno-fast-math -ffp-contract=off)
