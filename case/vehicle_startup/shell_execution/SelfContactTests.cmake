include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactSurfaceBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../joints/VehicleJointModel.cmake")
add_executable(robo_dyna_vehicle_self_contact_source_check
  "${CMAKE_CURRENT_LIST_DIR}/../physical_model/tests/supports/Source.cpp"
  tests/self_contact/Source.cpp tests/self_contact/CensusTest.cpp tests/self_contact/LimitsTest.cpp)
target_link_libraries(robo_dyna_vehicle_self_contact_source_check PRIVATE
  robo_dyna_vehicle_shell_execution robo_dyna_vehicle_joint_model
  tl_self_contact_surface_binding GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_self_contact_source_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_self_contact_source_original COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>" "VehicleSelfContactSource.*")
set_tests_properties(vehicle_self_contact_source_original PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 2
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
