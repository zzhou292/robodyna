include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/ObserverSupport.cmake")

add_executable(robo_dyna_vehicle_run_observed_check
  "${CMAKE_CURRENT_LIST_DIR}/ContributionTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/BudgetTest.cpp")
target_link_libraries(robo_dyna_vehicle_run_observed_check PRIVATE
  robo_dyna_vehicle_run_observed_qualification GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_run_observed_check PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_run_observed_values COMMAND robo_dyna_vehicle_run_observed_check)
set_tests_properties(vehicle_run_observed_values PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1 LABELS "unit;host;controller;failure-observer")
