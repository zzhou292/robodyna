include_guard(GLOBAL)
add_executable(robo_dyna_vehicle_failure_diagnostics_check "${CMAKE_CURRENT_LIST_DIR}/FailureRunTest.cpp")
target_link_libraries(robo_dyna_vehicle_failure_diagnostics_check PRIVATE
  robo_dyna_vehicle_failure_diagnostics GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_failure_diagnostics_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_run_failure_diagnostics COMMAND robo_dyna_vehicle_failure_diagnostics_check)
set_tests_properties(vehicle_run_failure_diagnostics PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE
  PROCESSORS 1 LABELS "unit;host;controller;failure-diagnostics")

option(ROBO_DYNA_VEHICLE_FAILURE_CUDA_COUPON "Build small native later-epoch diagnostic capture gate" OFF)
if(ROBO_DYNA_VEHICLE_FAILURE_CUDA_COUPON)
  include("${CMAKE_CURRENT_LIST_DIR}/LaterEpochCuda.cmake")
endif()
