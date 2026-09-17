include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactSurfaceBinding.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/FixedContactFacetBinding.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactCurrentRegularity.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/OriginalSelection.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_self_contact/VehicleSelfContactSetup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_self_contact/VehicleSelfContactInitialCensus.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_self_contact/VehicleSelfContactRuntime.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../joints/VehicleJointModel.cmake")
add_executable(robo_dyna_vehicle_self_contact_source_check
  "${CMAKE_CURRENT_LIST_DIR}/../physical_model/tests/supports/Source.cpp"
  tests/self_contact/Source.cpp tests/self_contact/CensusTest.cpp
  tests/self_contact/LimitsTest.cpp tests/self_contact/OriginalContactTest.cpp
  tests/self_contact/SetupTest.cpp tests/self_contact/InitialCensusTest.cpp
  tests/self_contact/RuntimeGateTest.cpp
  tests/self_contact/CandidateRigidCouponTest.cpp)
target_link_libraries(robo_dyna_vehicle_self_contact_source_check PRIVATE
  robo_dyna_vehicle_shell_execution robo_dyna_vehicle_joint_model
  robo_dyna_vehicle_self_contact_setup
  robo_dyna_vehicle_self_contact_initial_census
  robo_dyna_vehicle_self_contact_runtime
  robo_dyna_original_self_contact_selection
  tl_self_contact_surface_binding tl_fixed_contact_facets
  tl_self_contact_current_regularity GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_self_contact_source_check PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_vehicle_self_contact_nonlinear_fixture_check
  tests/self_contact/NonlinearCoverageFixtureTest.cpp)
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
  ROBO_NONLINEAR_FIXTURE_PATH="${CMAKE_CURRENT_LIST_DIR}/tests/self_contact/NonlinearAmbiguousFixture.bin")
add_test(NAME vehicle_self_contact_nonlinear_fixture
  COMMAND robo_dyna_vehicle_self_contact_nonlinear_fixture_check)
set_tests_properties(vehicle_self_contact_nonlinear_fixture PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
  LABELS "unit;nonlinear-fixture;accepted-ledger;bounded-subdivision")
add_executable(robo_dyna_vehicle_self_contact_linear_fixture_check
  tests/self_contact/LinearCoverageFixtureTest.cpp)
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
  ROBO_LINEAR_FIXTURE_PATH="${CMAKE_CURRENT_LIST_DIR}/tests/self_contact/LinearWorkExhaustedFixture.bin")
add_test(NAME vehicle_self_contact_linear_fixture
  COMMAND robo_dyna_vehicle_self_contact_linear_fixture_check)
set_tests_properties(vehicle_self_contact_linear_fixture PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
  LABELS "unit;linear-fixture;accepted-ledger;bounded-work")
add_test(NAME vehicle_self_contact_source_original COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>" "VehicleSelfContactSource.*")
set_tests_properties(vehicle_self_contact_source_original PROPERTIES TIMEOUT 1800 RUN_SERIAL TRUE PROCESSORS 2
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
add_test(NAME vehicle_self_contact_initial_census COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>"
  "VehicleSelfContactInitialCensus.*")
set_tests_properties(vehicle_self_contact_initial_census PROPERTIES
  TIMEOUT 180 RUN_SERIAL TRUE PROCESSORS 4
  LABELS "coupon;real-geometry;v5-exact-sample"
  RESOURCE_LOCK vehicle_self_contact_gpu
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")

add_test(NAME vehicle_self_contact_candidate_rigid_coupon
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>"
    "VehicleSelfContactCandidateCoupon.*")
set_tests_properties(vehicle_self_contact_candidate_rigid_coupon PROPERTIES
  TIMEOUT 600 RUN_SERIAL TRUE PROCESSORS 2
  LABELS "coupon;real-geometry;v5-candidate"
  RESOURCE_LOCK vehicle_self_contact_gpu
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")

add_test(NAME vehicle_self_contact_nonlinear_roster_coupon
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>"
    "VehicleSelfContactNonlinearRosterCoupon.*")
set_tests_properties(vehicle_self_contact_nonlinear_roster_coupon PROPERTIES
  TIMEOUT 1800 RUN_SERIAL TRUE PROCESSORS 24
  LABELS "coupon;real-geometry;v5-candidate;accepted-assembly;nonlinear-roster"
  RESOURCE_LOCK vehicle_self_contact_gpu
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")

add_test(NAME vehicle_self_contact_residual_translation_coupon
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>"
    "VehicleSelfContactAcceptedAssemblyCoupon.*")
set_tests_properties(
  vehicle_self_contact_residual_translation_coupon PROPERTIES
  TIMEOUT 1800 RUN_SERIAL TRUE PROCESSORS 2
  LABELS "coupon;real-geometry;v5-candidate;accepted-assembly"
  RESOURCE_LOCK vehicle_self_contact_gpu
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")

set(ROBO_DYNA_VEHICLE_WALL_MANIFEST "" CACHE FILEPATH
  "Pinned original canonical wall manifest for the combined runtime gate")
if(ROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE)
  set(_robo_self_contact_fixture_environment
    "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
  add_test(NAME vehicle_self_contact_acceptance_v5
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>"
      "VehicleSelfContactRuntime.FullV5SingleAttemptSealsAndDiscards")
  set_tests_properties(vehicle_self_contact_acceptance_v5 PROPERTIES
    TIMEOUT 7200 RUN_SERIAL TRUE PROCESSORS 2
    LABELS "acceptance-v5;large;V5_SELF_CONTACT_RUNTIME;GPU"
    RESOURCE_LOCK vehicle_self_contact_gpu
    ENVIRONMENT "${_robo_self_contact_fixture_environment}")

  if(EXISTS "${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
    add_test(NAME vehicle_wall_self_contact_acceptance_v5
      COMMAND "${Python3_EXECUTABLE}" -B
        "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
        "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
        "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>"
        "VehicleWallSelfContactRuntime.FullV5CombinedSingleAttemptSealsBothReceipts")
    set_tests_properties(vehicle_wall_self_contact_acceptance_v5 PROPERTIES
      TIMEOUT 7200 RUN_SERIAL TRUE PROCESSORS 2
      LABELS "acceptance-v5;large;V5_SELF_CONTACT_RUNTIME;GPU;WALL_SELF_CONTACT"
      RESOURCE_LOCK vehicle_self_contact_gpu
      ENVIRONMENT
        "${_robo_self_contact_fixture_environment};ROBO_VEHICLE_WALL=${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
  else()
    message(STATUS
      "Combined wall+self V5 acceptance tests are not registered: set ROBO_DYNA_VEHICLE_WALL_MANIFEST")
  endif()
  unset(_robo_self_contact_fixture_environment)
else()
  message(STATUS
    "Full V5 self-contact runtime tests are not registered: set ROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE=ON")
endif()
