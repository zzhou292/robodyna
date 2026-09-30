include_guard(GLOBAL)
# Shared frozen host replays use the current owning TL implementation. All
# source and fixture paths are relative to this module, independent of its caller.
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactTransaction.cmake")
add_executable(robo_dyna_vehicle_self_contact_nonlinear_fixture_check
  "${CMAKE_CURRENT_LIST_DIR}/NonlinearCoverageFixtureTest.cpp")
target_link_libraries(
  robo_dyna_vehicle_self_contact_nonlinear_fixture_check PRIVATE
  tl_self_contact_transaction
  tl_fixed_triangle_feature_discovery
  GTest::gtest_main)
target_compile_options(
  robo_dyna_vehicle_self_contact_nonlinear_fixture_check PRIVATE
  -fno-fast-math -ffp-contract=off)
target_compile_definitions(
  robo_dyna_vehicle_self_contact_nonlinear_fixture_check PRIVATE
  ROBO_NONLINEAR_FIXTURE_PATH="${CMAKE_CURRENT_LIST_DIR}/NonlinearAmbiguousFixture.bin")
add_test(NAME vehicle_self_contact_nonlinear_fixture
  COMMAND robo_dyna_vehicle_self_contact_nonlinear_fixture_check)
set_tests_properties(vehicle_self_contact_nonlinear_fixture PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
  LABELS "unit;nonlinear-fixture;accepted-ledger;bounded-subdivision")
add_executable(robo_dyna_vehicle_self_contact_linear_fixture_check
  "${CMAKE_CURRENT_LIST_DIR}/LinearCoverageFixtureTest.cpp")
target_link_libraries(
  robo_dyna_vehicle_self_contact_linear_fixture_check PRIVATE
  tl_self_contact_transaction
  tl_fixed_triangle_feature_discovery
  tl_represented_interval_crossing
  GTest::gtest_main)
target_compile_options(
  robo_dyna_vehicle_self_contact_linear_fixture_check PRIVATE
  -fno-fast-math -ffp-contract=off)
target_compile_definitions(
  robo_dyna_vehicle_self_contact_linear_fixture_check PRIVATE
  ROBO_LINEAR_FIXTURE_PATH="${CMAKE_CURRENT_LIST_DIR}/LinearWorkExhaustedFixture.bin")
add_test(NAME vehicle_self_contact_linear_fixture
  COMMAND robo_dyna_vehicle_self_contact_linear_fixture_check)
set_tests_properties(vehicle_self_contact_linear_fixture PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
  LABELS "unit;linear-fixture;accepted-ledger;bounded-work")
