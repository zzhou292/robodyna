set(ROBO_DYNA_CONNECTIVITY_V5_REPORT "" CACHE FILEPATH "Optional create-only complete V5 connectivity report")
add_executable(robo_dyna_vehicle_connectivity_v5_check
  "${CMAKE_CURRENT_LIST_DIR}/../physical_model/tests/supports/Source.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tests/V5Source.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tests/V5RelationsTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tests/V5ReachabilityTest.cpp")
target_link_libraries(robo_dyna_vehicle_connectivity_v5_check PRIVATE
  robo_dyna_vehicle_connectivity GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_connectivity_v5_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_connectivity_v5 COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../tied_classification/tests/actual_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
  "$<TARGET_FILE:robo_dyna_vehicle_connectivity_v5_check>" "VehicleConnectivityV5.*")
set_tests_properties(vehicle_connectivity_v5 PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 2
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION};ROBO_DYNA_CONNECTIVITY_V5_REPORT=${ROBO_DYNA_CONNECTIVITY_V5_REPORT}")
