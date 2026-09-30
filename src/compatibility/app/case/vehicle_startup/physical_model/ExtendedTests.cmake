option(ROBO_DYNA_VEHICLE_EXTENDED_MODEL_TESTS "Original4900 complete physical source/domain/model gate" OFF)
if(ROBO_DYNA_VEHICLE_EXTENDED_MODEL_TESTS)
  include("${CMAKE_CURRENT_LIST_DIR}/../joints/VehicleJointModel.cmake")
  add_executable(robo_dyna_vehicle_extended_model_check tests/extended/Source.cpp
    tests/extended/DomainTest.cpp tests/extended/ModelTest.cpp
    tests/extended/CoefficientTest.cpp tests/extended/RejectionTest.cpp)
  target_link_libraries(robo_dyna_vehicle_extended_model_check PRIVATE robo_dyna_vehicle_joint_model GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_extended_model_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_extended_physical_model_original COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_vehicle_extended_model_check>" "VehicleExtendedPhysicalOriginal.*")
  set_tests_properties(vehicle_extended_physical_model_original PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
endif()
