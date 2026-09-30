# Explicit source-only rear mount admission; no case/runtime policy changes.
add_executable(robo_dyna_vehicle_rear_solid_source_check tests/RearSourceTest.cpp)
target_link_libraries(robo_dyna_vehicle_rear_solid_source_check PRIVATE
  robo_dyna_vehicle_solid_source GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_rear_solid_source_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_rear_solid_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_rear_solid_source_check>" "VehicleRearSolidSource.*")
