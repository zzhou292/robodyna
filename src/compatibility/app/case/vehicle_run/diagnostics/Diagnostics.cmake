include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../tests/observed/ObserverSupport.cmake")
add_library(robo_dyna_vehicle_failure_diagnostics STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Destination.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Publication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FailureRun.cpp")
target_link_libraries(robo_dyna_vehicle_failure_diagnostics PUBLIC robo_dyna_vehicle_run
  PRIVATE robo_dyna_vehicle_run_observed_qualification)
target_compile_features(robo_dyna_vehicle_failure_diagnostics PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_failure_diagnostics PRIVATE -fno-fast-math -ffp-contract=off)

# Registration remains separate so production library includes never create tests.
