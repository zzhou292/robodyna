include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/Values.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_wall/VehicleWallStartup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_self_contact/VehicleSelfContactRuntime.cmake")
set(ROBO_DYNA_PHYSICAL_RUN_LIVE_FACTORY ON)
set(ROBO_DYNA_PHYSICAL_RUN_WALL ON)
include("${CMAKE_CURRENT_LIST_DIR}/RunReports.cmake")
add_library(robo_dyna_vehicle_run STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Prepare.cpp" "${CMAKE_CURRENT_LIST_DIR}/Session.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Execute.cpp" "${CMAKE_CURRENT_LIST_DIR}/ContactComposition.cpp")
target_link_libraries(robo_dyna_vehicle_run PUBLIC robo_dyna_vehicle_run_values robo_dyna_vehicle_run_reports
  robo_dyna_vehicle_loaded_wall robo_dyna_physical_run_wall robo_dyna_physical_run_live
  robo_dyna_vehicle_self_contact_runtime)
target_compile_features(robo_dyna_vehicle_run PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_run PRIVATE -fno-fast-math -ffp-contract=off)

add_library(robo_dyna_vehicle_run_original_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/source/OriginalSources.cpp" "${CMAKE_CURRENT_LIST_DIR}/source/OriginalYaris.cpp")
target_link_libraries(robo_dyna_vehicle_run_original_source PUBLIC robo_dyna_vehicle_run)
target_compile_options(robo_dyna_vehicle_run_original_source PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_vehicle_run_cli "${CMAKE_CURRENT_LIST_DIR}/cli/Main.cpp")
set_target_properties(robo_dyna_vehicle_run_cli PROPERTIES OUTPUT_NAME robo_dyna_vehicle_run)
target_link_libraries(robo_dyna_vehicle_run_cli PRIVATE robo_dyna_vehicle_run_original_source)
