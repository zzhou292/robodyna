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
  tests/self_contact/RuntimeGateTest.cpp)
target_link_libraries(robo_dyna_vehicle_self_contact_source_check PRIVATE
  robo_dyna_vehicle_shell_execution robo_dyna_vehicle_joint_model
  robo_dyna_vehicle_self_contact_setup
  robo_dyna_vehicle_self_contact_initial_census
  robo_dyna_vehicle_self_contact_runtime
  robo_dyna_original_self_contact_selection
  tl_self_contact_surface_binding tl_fixed_contact_facets
  tl_self_contact_current_regularity GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_self_contact_source_check PRIVATE -fno-fast-math -ffp-contract=off)
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
  TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 2
  LABELS "coupon;real-geometry;v5-filter-census"
  RESOURCE_LOCK vehicle_self_contact_gpu
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")

set(ROBO_DYNA_VEHICLE_WALL_MANIFEST "" CACHE FILEPATH
  "Pinned original canonical wall manifest for the combined runtime gate")
if(ROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE)
  set(_robo_self_contact_fixture_environment
    "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
  foreach(runtime_gate IN ITEMS startup one_attempt)
    if(runtime_gate STREQUAL "startup")
      set(runtime_filter
        "VehicleSelfContactRuntime.FullV5ForecastStartupOwnsExactIdentityAndMemory")
      set(runtime_timeout 3600)
    else()
      set(runtime_filter
        "VehicleSelfContactRuntime.FullV5OneAttemptIsTypedFailClosedAndRetryStable")
      set(runtime_timeout 21600)
    endif()
    add_test(NAME vehicle_self_contact_runtime_${runtime_gate}
      COMMAND "${Python3_EXECUTABLE}" -B
        "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
        "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
        "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>"
        "${runtime_filter}")
    set_tests_properties(
      vehicle_self_contact_runtime_${runtime_gate} PROPERTIES
      TIMEOUT ${runtime_timeout} RUN_SERIAL TRUE PROCESSORS 2
      LABELS "acceptance-v5;V5_SELF_CONTACT_RUNTIME;GPU"
      RESOURCE_LOCK vehicle_self_contact_gpu
      ENVIRONMENT "${_robo_self_contact_fixture_environment}")
  endforeach()

  if(EXISTS "${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
    foreach(runtime_gate IN ITEMS startup one_attempt)
      if(runtime_gate STREQUAL "startup")
        set(runtime_filter
          "VehicleWallSelfContactRuntime.FullV5CombinedForecastStartupOwnsBothFixedSlotsOnce")
        set(runtime_timeout 3600)
      else()
        set(runtime_filter
          "VehicleWallSelfContactRuntime.FullV5CombinedAttemptSealsBothReceiptsAndRetries")
        set(runtime_timeout 21600)
      endif()
      add_test(NAME vehicle_wall_self_contact_runtime_${runtime_gate}
        COMMAND "${Python3_EXECUTABLE}" -B
          "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/tests/actual_fixture.py"
          "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
          "$<TARGET_FILE:robo_dyna_vehicle_self_contact_source_check>"
          "${runtime_filter}")
      set_tests_properties(
        vehicle_wall_self_contact_runtime_${runtime_gate} PROPERTIES
        TIMEOUT ${runtime_timeout} RUN_SERIAL TRUE PROCESSORS 2
        LABELS "acceptance-v5;V5_SELF_CONTACT_RUNTIME;GPU;WALL_SELF_CONTACT"
        RESOURCE_LOCK vehicle_self_contact_gpu
        ENVIRONMENT
          "${_robo_self_contact_fixture_environment};ROBO_VEHICLE_WALL=${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")
    endforeach()
  else()
    message(STATUS
      "Combined wall+self V5 acceptance tests are not registered: set ROBO_DYNA_VEHICLE_WALL_MANIFEST")
  endif()
  unset(_robo_self_contact_fixture_environment)
else()
  message(STATUS
    "Full V5 self-contact runtime tests are not registered: set ROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE=ON")
endif()
