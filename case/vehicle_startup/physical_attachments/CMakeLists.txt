cmake_minimum_required(VERSION 3.22)
project(RoboDynaVehiclePhysicalAttachments LANGUAGES CXX CUDA)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalAttachments.cmake")
foreach(name CANONICAL SCOPE DECLARATIONS GLASS_RESOLUTION TYPE13_DECLARATION)
  set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
  if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
    message(FATAL_ERROR "Physical attachment checks require ${name}")
  endif()
endforeach()
set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated original glass sidecar SHA256")
string(LENGTH "${ROBO_DYNA_VEHICLE_GLASS_SHA256}" sha_length)
if(NOT sha_length EQUAL 64)
  message(FATAL_ERROR "Physical attachment checks require glass SHA256")
endif()
add_executable(robo_dyna_vehicle_physical_attachments_check tests/Source.cpp
  tests/OriginalAttachmentTest.cpp tests/RejectionTest.cpp)
target_link_libraries(robo_dyna_vehicle_physical_attachments_check PRIVATE
  robo_dyna_vehicle_physical_attachments GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_physical_attachments_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_physical_attachments_original COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../tied_classification/tests/actual_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
  "$<TARGET_FILE:robo_dyna_vehicle_physical_attachments_check>" "VehiclePhysicalAttachmentsOriginal.*")
set_tests_properties(vehicle_physical_attachments_original PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 2
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
