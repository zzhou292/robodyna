set(ROBO_DYNA_LAW90_SDI_OBSERVATION "" CACHE FILEPATH "Verified native original SDI observation")
if(NOT EXISTS "${ROBO_DYNA_LAW90_SDI_OBSERVATION}")
  message(FATAL_ERROR "Radiator source gate requires original native SDI observation")
endif()
add_executable(robo_dyna_vehicle_radiator_solid_source_check tests/RadiatorSourceTest.cpp)
target_link_libraries(robo_dyna_vehicle_radiator_solid_source_check PRIVATE
  robo_dyna_vehicle_solid_source GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_radiator_solid_source_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_radiator_solid_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_radiator_solid_source_check>" "VehicleRadiatorSolidSource.*")
set_tests_properties(vehicle_radiator_solid_source PROPERTIES
  ENVIRONMENT "ROBO_LAW90_NATIVE_OBSERVATION=${ROBO_DYNA_LAW90_SDI_OBSERVATION}")
