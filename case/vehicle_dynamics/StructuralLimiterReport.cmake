include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_wall/VehicleWallStartup.cmake")
add_library(robo_dyna_structural_limiter_report STATIC "${CMAKE_CURRENT_LIST_DIR}/StructuralLimiterReport.cpp")
target_link_libraries(robo_dyna_structural_limiter_report PUBLIC
  robo_dyna_vehicle_physical_dynamics robo_dyna_vehicle_wall_setup)
target_compile_features(robo_dyna_structural_limiter_report PUBLIC cxx_std_17)
target_compile_options(robo_dyna_structural_limiter_report PRIVATE -fno-fast-math -ffp-contract=off)
