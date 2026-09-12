add_executable(robo_dyna_vehicle_airbag_solid_source_check tests/AirbagSourceTest.cpp)
target_link_libraries(robo_dyna_vehicle_airbag_solid_source_check PRIVATE
  robo_dyna_vehicle_solid_source GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_airbag_solid_source_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_airbag_solid_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_airbag_solid_source_check>" "VehicleAirbagSolidSource.*")
add_test(NAME vehicle_airbag_source_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/solid_law44_analytic/verify_source.py")
