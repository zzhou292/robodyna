option(ROBO_DYNA_VEHICLE_SUPPORTS_MODEL_TESTS "Original4980+142 complete vehicle support source/model gate" OFF)
option(ROBO_DYNA_VEHICLE_SUPPORTS_CIN_TESTS "Original V5 CIN proof and host role packing gate" OFF)
if(ROBO_DYNA_VEHICLE_SUPPORTS_MODEL_TESTS OR ROBO_DYNA_VEHICLE_SUPPORTS_CIN_TESTS)
  include("${CMAKE_CURRENT_LIST_DIR}/../joints/VehicleJointModel.cmake")
  add_executable(robo_dyna_vehicle_supports_model_check tests/supports/Source.cpp
    tests/supports/DomainTest.cpp tests/supports/SolidTest.cpp tests/supports/ModelTest.cpp tests/supports/JointTest.cpp
    tests/supports/EvidenceTest.cpp tests/supports/RejectionTest.cpp)
  target_link_libraries(robo_dyna_vehicle_supports_model_check PRIVATE robo_dyna_vehicle_joint_model GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_supports_model_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_supports_physical_model_original COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_vehicle_supports_model_check>" "VehicleSupportsPhysicalOriginal.*")
  set_tests_properties(vehicle_supports_physical_model_original PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
endif()
if(ROBO_DYNA_VEHICLE_SUPPORTS_CIN_TESTS)
  enable_language(CUDA)
  include("${CMAKE_CURRENT_LIST_DIR}/../physical_attachments/VehiclePhysicalAttachments.cmake")
  add_executable(robo_dyna_vehicle_supports_cin_check tests/supports/Source.cpp tests/supports/AttachmentTest.cpp
    "${CMAKE_CURRENT_LIST_DIR}/../../vehicle_runtime/SourceRoles.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../vehicle_runtime/Packing.cpp")
  target_link_libraries(robo_dyna_vehicle_supports_cin_check PRIVATE robo_dyna_vehicle_physical_attachments
    robo_dyna_vehicle_joint_model GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_supports_cin_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_supports_physical_cin_original COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../tied_classification/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
    "$<TARGET_FILE:robo_dyna_vehicle_supports_cin_check>" "VehicleSupportsPhysicalAttachments.*")
  set_tests_properties(vehicle_supports_physical_cin_original PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 2
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
endif()
