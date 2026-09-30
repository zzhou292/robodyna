# Source population and ownership only; no mechanical factory/runtime enable.
include("${CMAKE_CURRENT_LIST_DIR}/../point_mass/VehiclePointMassSource.cmake")
add_executable(robo_dyna_native_v6_population_fields_check
  "${CMAKE_CURRENT_LIST_DIR}/../physical_scope/tests/FieldsTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_domain/tests/SelectionTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_domain/tests/ProfileTest.cpp")
target_link_libraries(robo_dyna_native_v6_population_fields_check PRIVATE robo_dyna_vehicle_physical_domain GTest::gtest_main)
target_compile_options(robo_dyna_native_v6_population_fields_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME native_v6_population_fields COMMAND robo_dyna_native_v6_population_fields_check)
add_executable(robo_dyna_native_v6_population_source_check tests/NativePopulationTest.cpp)
target_link_libraries(robo_dyna_native_v6_population_source_check PRIVATE robo_dyna_vehicle_type45_source robo_dyna_vehicle_point_mass_source GTest::gtest_main)
target_compile_options(robo_dyna_native_v6_population_source_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME native_v6_population_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_PHYSICAL_CANONICAL}" "${ROBO_DYNA_PHYSICAL_SCOPE}"
  "$<TARGET_FILE:robo_dyna_native_v6_population_source_check>" "VehicleNativeV6Population.*")
set_tests_properties(native_v6_population_source PROPERTIES ENVIRONMENT
 "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_PHYSICAL_DECLARATIONS};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_PHYSICAL_TYPE13_DECLARATION}")
