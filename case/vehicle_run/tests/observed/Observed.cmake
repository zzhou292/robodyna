include_guard(GLOBAL)
# Test-only library. The CLI links the ordinary controller, never this bridge.
add_library(robo_dyna_vehicle_run_observed_qualification STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Contribution.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NativeSeal.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RunAccess.cpp")
target_link_libraries(robo_dyna_vehicle_run_observed_qualification PUBLIC
  robo_dyna_vehicle_run robo_dyna_candidate_failure_fixture)
target_compile_features(robo_dyna_vehicle_run_observed_qualification PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_run_observed_qualification PRIVATE
  -fno-fast-math -ffp-contract=off)
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
