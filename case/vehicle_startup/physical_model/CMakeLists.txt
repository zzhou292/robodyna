cmake_minimum_required(VERSION 3.18)
project(RoboDynaVehiclePhysicalModel LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalModel.cmake")
foreach(name CANONICAL SCOPE DECLARATIONS GLASS_RESOLUTION TYPE13_DECLARATION)
  set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
  if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
    message(FATAL_ERROR "Physical model checks require ${name}")
  endif()
endforeach()
set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated original glass sidecar SHA256")
string(LENGTH "${ROBO_DYNA_VEHICLE_GLASS_SHA256}" sha_length)
if(NOT sha_length EQUAL 64)
  message(FATAL_ERROR "Physical model checks require glass SHA256")
endif()
add_executable(robo_dyna_vehicle_physical_model_check tests/OriginalModelTest.cpp
  tests/CoefficientTest.cpp tests/RejectionTest.cpp)
target_link_libraries(robo_dyna_vehicle_physical_model_check PRIVATE robo_dyna_vehicle_physical_model GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_physical_model_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_physical_model_original COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_physical_model_check>" "VehiclePhysicalModelOriginal.*")
set_tests_properties(vehicle_physical_model_original PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")

add_executable(robo_dyna_vehicle_rear_model_check tests/RearSolidModelTest.cpp)
target_link_libraries(robo_dyna_vehicle_rear_model_check PRIVATE robo_dyna_vehicle_physical_model GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_rear_model_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_rear_solid_model COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_rear_model_check>" "VehicleRearSolidModel.*")
set_tests_properties(vehicle_rear_solid_model PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1)
