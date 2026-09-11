if(NOT ROBO_DYNA_PHYSICAL_RUN_LIVE_FACTORY)
  message(FATAL_ERROR "Original run archive gate requires actual live factory")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
foreach(name CANONICAL SCOPE DECLARATIONS GLASS_RESOLUTION TYPE13_DECLARATION)
  set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name}")
  if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
    message(FATAL_ERROR "Physical run archive requires ${name}")
  endif()
endforeach()
set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated glass sidecar SHA256")
add_executable(robo_dyna_physical_run_original_check tests/OriginalTest.cpp
  "${CMAKE_CURRENT_LIST_DIR}/../../case/vehicle_startup/physical_attachments/tests/Source.cpp")
target_link_libraries(robo_dyna_physical_run_original_check PRIVATE robo_dyna_physical_run_live GTest::gtest_main)
target_compile_options(robo_dyna_physical_run_original_check PRIVATE -fno-fast-math -ffp-contract=off)
foreach(gate InitialOnlyPrefixHasCompleteSourceAndNoAcceptedIntervalClaim OneActualAcceptedIntervalFactoryDiscardAndFailedPrefixRoundTrip
    OriginalJointInitialPrefixAuthenticatesSeventhSourceAndVirginPhase)
  add_test(NAME physical_run_${gate} COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../case/vehicle_startup/tied_classification/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
    "$<TARGET_FILE:robo_dyna_physical_run_original_check>" "PhysicalRunOriginal.${gate}")
  set_tests_properties(physical_run_${gate} PROPERTIES TIMEOUT 600 RUN_SERIAL TRUE PROCESSORS 2
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
endforeach()
