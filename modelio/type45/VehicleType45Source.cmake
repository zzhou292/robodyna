include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_domain/VehiclePhysicalDomain.cmake")
add_library(robo_dyna_vehicle_type45_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehicleType45Source.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Fields.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Mapping.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp")
target_link_libraries(robo_dyna_vehicle_type45_source PUBLIC robo_dyna_vehicle_physical_domain)
target_compile_features(robo_dyna_vehicle_type45_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_type45_source PRIVATE -fno-fast-math -ffp-contract=off)
