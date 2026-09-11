include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/VehicleWallValues.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_dynamics/VehiclePhysicalDynamics.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../shell_collection/ShellCollectionContactGeometry.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../PlacedCanonicalWall.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/NodalWallMappedContact.cmake")
add_library(robo_dyna_vehicle_wall_setup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SetupForecast.cpp" "${CMAKE_CURRENT_LIST_DIR}/VehicleWallSetup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Artifacts.cpp")
target_link_libraries(robo_dyna_vehicle_wall_setup PUBLIC robo_dyna_vehicle_wall_values
  robo_dyna_vehicle_physical_startup robo_dyna_shell_collection_contact_geometry robo_dyna_placed_canonical_wall)
target_compile_options(robo_dyna_vehicle_wall_setup PRIVATE -fno-fast-math -ffp-contract=off)
add_library(robo_dyna_vehicle_wall_startup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/RuntimePreview.cpp" "${CMAKE_CURRENT_LIST_DIR}/VehicleWallStartup.cpp")
target_link_libraries(robo_dyna_vehicle_wall_startup PUBLIC robo_dyna_vehicle_wall_setup
  robo_dyna_vehicle_physical_dynamics tl_nodal_wall_mapped)
target_compile_options(robo_dyna_vehicle_wall_startup PRIVATE -fno-fast-math -ffp-contract=off)
