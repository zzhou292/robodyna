find_package(Python3 REQUIRED COMPONENTS Interpreter)
include("${CMAKE_CURRENT_LIST_DIR}/VehicleConnectivity.cmake")
foreach(name CANONICAL SCOPE DECLARATIONS GLASS_RESOLUTION TYPE13_DECLARATION)
  set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
  if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
    message(FATAL_ERROR "Connectivity original gate requires ${name}")
  endif()
endforeach()
set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated original glass sidecar SHA256")
string(LENGTH "${ROBO_DYNA_VEHICLE_GLASS_SHA256}" sha_length)
if(NOT sha_length EQUAL 64)
  message(FATAL_ERROR "Connectivity original gate requires glass SHA256")
endif()
set(ROBO_DYNA_CONNECTIVITY_REPORT "" CACHE FILEPATH "Optional create-only complete connectivity report")
if(ROBO_DYNA_CONNECTIVITY_ORIGINAL)
add_executable(robo_dyna_vehicle_connectivity_original_check
  "${CMAKE_CURRENT_LIST_DIR}/../physical_attachments/tests/Source.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tests/OriginalTest.cpp")
target_link_libraries(robo_dyna_vehicle_connectivity_original_check PRIVATE
  robo_dyna_vehicle_connectivity GTest::gtest_main)
add_test(NAME vehicle_connectivity_original COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../tied_classification/tests/actual_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
  "$<TARGET_FILE:robo_dyna_vehicle_connectivity_original_check>" "VehicleConnectivityOriginal.*")
set_tests_properties(vehicle_connectivity_original PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 2
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION};ROBO_DYNA_CONNECTIVITY_REPORT=${ROBO_DYNA_CONNECTIVITY_REPORT}")
endif()
if(ROBO_DYNA_CONNECTIVITY_SUPPORTS)
  include("${CMAKE_CURRENT_LIST_DIR}/V5.cmake")
endif()
