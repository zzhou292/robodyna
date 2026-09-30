cmake_minimum_required(VERSION 3.22)
project(RoboDynaVehicleAcceptedFrames LANGUAGES CXX CUDA)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
include("${CMAKE_CURRENT_LIST_DIR}/VehicleAcceptedFrames.cmake")
foreach(name CANONICAL SCOPE DECLARATIONS GLASS_RESOLUTION TYPE13_DECLARATION)
  set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name}")
  if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
    message(FATAL_ERROR "Dynamics capture requires ${name}")
  endif()
endforeach()
set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated glass sidecar SHA256")
add_executable(robo_dyna_vehicle_dynamics_capture_check tests/OriginalTest.cpp
  "${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/physical_attachments/tests/Source.cpp")
target_link_libraries(robo_dyna_vehicle_dynamics_capture_check PRIVATE robo_dyna_vehicle_accepted_frames GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_dynamics_capture_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_dynamics_accepted_capture COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/tied_classification/tests/actual_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
  "$<TARGET_FILE:robo_dyna_vehicle_dynamics_capture_check>"
  "VehicleDynamicsCaptureOriginal.AcceptedEpochReadbackDiscardAndBinaryRoundTrip")
set_tests_properties(vehicle_dynamics_accepted_capture PROPERTIES TIMEOUT 600 RUN_SERIAL TRUE PROCESSORS 2
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
