if(NOT ROBO_DYNA_VEHICLE_RUN_LIVE)
  message(FATAL_ERROR "Original run gate requires the live controller")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_runtime" startup)
set(ROBO_DYNA_VEHICLE_WALL_MANIFEST "" CACHE FILEPATH "Pinned original canonical wall manifest")
if(NOT EXISTS "${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
  message(FATAL_ERROR "Original run needs its pinned wall manifest")
endif()
add_executable(robo_dyna_vehicle_run_original_check
  "${CMAKE_CURRENT_LIST_DIR}/OriginalTest.cpp" "${CMAKE_CURRENT_LIST_DIR}/SupportsTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/LimiterTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ContactCompositionTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SelfContactTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SecondIntervalCensusTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SelfContactFailureCaptureTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/shell_execution/tests/self_contact/CoverageFixtureValuesTest.cpp")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/shell_execution/tests/self_contact/CoverageFixtureCapture.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/shell_execution/tests/self_contact/PreparedCensusReplay.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/shell_execution/tests/self_contact/CandidateFailureFixture.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/observed/Observed.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_dynamics/StructuralLimiterReport.cmake")
target_link_libraries(robo_dyna_vehicle_run_original_check PRIVATE robo_dyna_vehicle_run_original_source GTest::gtest_main)
target_link_libraries(robo_dyna_vehicle_run_original_check PRIVATE robo_dyna_structural_limiter_report)
target_link_libraries(robo_dyna_vehicle_run_original_check PRIVATE robo_dyna_self_contact_fixture_capture robo_dyna_candidate_failure_fixture)
add_test(NAME vehicle_limiter_values COMMAND robo_dyna_vehicle_run_original_check
  --gtest_filter=VehicleLimiterValues.*)
add_test(NAME vehicle_run_contact_composition COMMAND robo_dyna_vehicle_run_original_check
  --gtest_filter=VehicleContactComposition.*)
set_tests_properties(vehicle_run_contact_composition PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1 LABELS "unit;controller;contact-composition")
add_test(NAME vehicle_self_contact_fixture_capture_values
  COMMAND robo_dyna_vehicle_run_original_check --gtest_filter=VehicleSelfContactFixtureCapture.*)
set_tests_properties(vehicle_self_contact_fixture_capture_values PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1 LABELS "unit;self-contact;fixture-capture")
target_compile_options(robo_dyna_vehicle_run_original_check PRIVATE -fno-fast-math -ffp-contract=off)
foreach(gate forecast loaded_prefix extended_forecast extended_loaded_prefix supports_forecast supports_initial supports_loaded_prefix supports_limiter)
  if(gate STREQUAL forecast)
    set(filter "VehicleRunOriginal.ForecastRetainsCompleteSourceAndRejectsStaleRunIdentityBeforeOwner")
  elseif(gate STREQUAL extended_forecast)
    set(filter "VehicleRunExtended.Full4900SourceForecastKeepsOriginalConnectionsAndNormalResourceCaps")
  elseif(gate STREQUAL extended_loaded_prefix)
    set(filter "VehicleRunExtended.TwoActualLoadedIntervalsKeep4900SolidOwnerAndAuthenticReplay")
  elseif(gate STREQUAL supports_forecast)
    set(filter "VehicleRunSupports.FullSupportForecastUsesActualBeamModelAndInclusiveCaps")
  elseif(gate STREQUAL supports_initial)
    set(filter "VehicleRunSupports.CompleteInitialOwnerReadsEightParticipantsAndPreservesFailedReplacement")
  elseif(gate STREQUAL supports_loaded_prefix)
    set(filter "VehicleRunSupports.TwoLoadedIntervalsKeepCompleteSupportsAndAuthenticatedReplay")
  elseif(gate STREQUAL supports_limiter)
    set(filter "VehicleRunSupports.ActualSuccessfulLimiterRetainsSourceThroughDiscardRetryAndAcceptance")
  else()
    set(filter "VehicleRunOriginal.TwoActualLoadedIntervalsExportAuthenticAcceptedPrefixAndViewerInput")
  endif()
  add_test(NAME vehicle_run_${gate} COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/tied_classification/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
    "$<TARGET_FILE:robo_dyna_vehicle_run_original_check>" "${filter}")
  set_tests_properties(vehicle_run_${gate} PROPERTIES TIMEOUT 600 RUN_SERIAL TRUE PROCESSORS 2
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION};ROBO_VEHICLE_WALL=${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
endforeach()

# Deliberately separate from the ordinary two-step wall-only gates. This full
# geometry traversal is final acceptance, not a development retry/coupon.
option(ROBO_DYNA_ENABLE_V5_SELF_CONTACT_CONTROLLER
  "Register expensive two-interval V5 wall+self controller acceptance" OFF)
if(ROBO_DYNA_ENABLE_V5_SELF_CONTACT_CONTROLLER)
  add_test(NAME vehicle_run_wall_self_contact_failure_capture
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_vehicle_run_original_check>"
      "VehicleRunWallSelfContactFailure.CaptureFirstNativeRejectionWithinTwoIntervals")
  set_tests_properties(vehicle_run_wall_self_contact_failure_capture PROPERTIES
    TIMEOUT 7200 RUN_SERIAL TRUE PROCESSORS 2
    LABELS "diagnostic;acceptance-v5;wall-self-contact;GPU;failure-capture"
    RESOURCE_LOCK vehicle_self_contact_gpu
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION};ROBO_VEHICLE_WALL=${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
  add_test(NAME vehicle_run_wall_self_contact_second_census
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_vehicle_run_original_check>"
      "VehicleRunWallSelfContactCensus.CommittedFirstIntervalThenCompleteActualSecondCandidateCensus")
  set_tests_properties(vehicle_run_wall_self_contact_second_census PROPERTIES
    TIMEOUT 7200 RUN_SERIAL TRUE PROCESSORS 2
    LABELS "diagnostic;acceptance-v5;wall-self-contact;GPU;second-candidate-census"
    RESOURCE_LOCK vehicle_self_contact_gpu
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION};ROBO_VEHICLE_WALL=${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
  add_test(NAME vehicle_run_wall_self_contact_forecast
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_vehicle_run_original_check>"
      "VehicleRunWallSelfContact.FullV5ForecastAdmitsEventHeadroomBeforeAnyDynamicsOwner")
  set_tests_properties(vehicle_run_wall_self_contact_forecast PROPERTIES
    TIMEOUT 600 RUN_SERIAL TRUE PROCESSORS 2
    LABELS "forecast;controller;wall-self-contact;GPU-startup"
    RESOURCE_LOCK vehicle_self_contact_gpu
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION};ROBO_VEHICLE_WALL=${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
  add_test(NAME vehicle_run_wall_self_contact_two_intervals
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_vehicle_run_original_check>"
      "VehicleRunWallSelfContact.TwoCommittedV5IntervalsPreserveBothContactProfilesAndReplay")
  set_tests_properties(vehicle_run_wall_self_contact_two_intervals PROPERTIES
    TIMEOUT 7200 RUN_SERIAL TRUE PROCESSORS 2
    LABELS "acceptance-v5;controller;wall-self-contact;GPU"
    RESOURCE_LOCK vehicle_self_contact_gpu
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION};ROBO_VEHICLE_WALL=${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
endif()
