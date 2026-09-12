include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_runtime/VehiclePhysicalStartup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/TiedCinWitnessActivity.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../benchmarks/stage_timing/StageTiming.cmake")
add_library(robo_dyna_vehicle_physical_dynamics STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalDynamics.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NativeStageError.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Trial.cpp" "${CMAKE_CURRENT_LIST_DIR}/MotionSummary.cpp")
target_link_libraries(robo_dyna_vehicle_physical_dynamics PUBLIC
  robo_dyna_vehicle_physical_startup robo_dyna_tied_cin_witness_activity robo_dyna_stage_timing)
target_compile_features(robo_dyna_vehicle_physical_dynamics PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_physical_dynamics PRIVATE -fno-fast-math -ffp-contract=off)
