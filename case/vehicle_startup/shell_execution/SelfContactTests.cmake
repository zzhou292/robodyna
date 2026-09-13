include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactSurfaceBinding.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/FixedContactFacetBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/OriginalSelection.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_self_contact/VehicleSelfContactSetup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../joints/VehicleJointModel.cmake")
add_executable(robo_dyna_vehicle_self_contact_source_check
  "${CMAKE_CURRENT_LIST_DIR}/../physical_model/tests/supports/Source.cpp"
  tests/self_contact/Source.cpp tests/self_contact/CensusTest.cpp
  tests/self_contact/LimitsTest.cpp tests/self_contact/OriginalContactTest.cpp
  tests/self_contact/SetupTest.cpp)
target_link_libraries(robo_dyna_vehicle_self_contact_source_check PRIVATE
  robo_dyna_vehicle_shell_execution robo_dyna_vehicle_joint_model
  robo_dyna_vehicle_self_contact_setup
  robo_dyna_original_self_contact_selection
  tl_self_contact_surface_binding tl_fixed_contact_facets GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_self_contact_source_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_self_contact_source_original COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>" "VehicleSelfContactSource.*")
set_tests_properties(vehicle_self_contact_source_original PROPERTIES TIMEOUT 1800 RUN_SERIAL TRUE PROCESSORS 2
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
