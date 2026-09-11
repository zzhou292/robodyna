include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../tied_shell/TiedShellDeclaration.cmake")
add_library(robo_dyna_vehicle_solid_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehicleSolidSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Sources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Cards.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Materials.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RearMaterial.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Geometry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/References.cpp")
target_link_libraries(robo_dyna_vehicle_solid_source PUBLIC robo_dyna_tied_shell_declaration)
target_compile_features(robo_dyna_vehicle_solid_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_solid_source PRIVATE -fno-fast-math -ffp-contract=off)
