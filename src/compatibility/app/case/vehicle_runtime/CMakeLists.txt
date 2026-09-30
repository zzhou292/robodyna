cmake_minimum_required(VERSION 3.22)
project(RoboDynaVehiclePhysicalStartup LANGUAGES CXX CUDA)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(CUDAToolkit REQUIRED)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalStartup.cmake")
add_executable(robo_dyna_vehicle_runtime_values_check tests/PackingTest.cpp
  "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/rigid_assembly_owner/Fixture.cpp")
target_link_libraries(robo_dyna_vehicle_runtime_values_check PRIVATE robo_dyna_vehicle_runtime_values
  tl_tied_cin_attachment GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_runtime_values_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_runtime_values COMMAND robo_dyna_vehicle_runtime_values_check)
foreach(name CANONICAL SCOPE DECLARATIONS GLASS_RESOLUTION TYPE13_DECLARATION)
  set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
  if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
    message(FATAL_ERROR "Physical startup checks require ${name}")
  endif()
endforeach()
set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated original glass sidecar SHA256")
string(LENGTH "${ROBO_DYNA_VEHICLE_GLASS_SHA256}" sha_length)
if(NOT sha_length EQUAL 64)
  message(FATAL_ERROR "Physical startup checks require glass SHA256")
endif()
add_executable(robo_dyna_vehicle_physical_startup_check tests/OriginalTest.cpp tests/EndpointRoleTest.cpp tests/JointOriginalTest.cpp
  "${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/physical_attachments/tests/Source.cpp")
target_link_libraries(robo_dyna_vehicle_physical_startup_check PRIVATE robo_dyna_vehicle_physical_startup GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_physical_startup_check PRIVATE -fno-fast-math -ffp-contract=off)
# Execute the forecast separately and inspect its complete source/device preview
# before authorizing this executable's original_owner CUDA allocation gate.
foreach(gate forecast original_owner joint_forecast joint_owner)
  if(gate STREQUAL forecast)
    set(filter "VehiclePhysicalRuntimeOriginal.ForecastAndSourceRoles:VehiclePhysicalAttachmentsOriginal.*")
  elseif(gate STREQUAL original_owner)
    set(filter "VehiclePhysicalRuntimeOriginal.CompleteInitialOwnerAndTypedReadback")
  elseif(gate STREQUAL joint_forecast)
    set(filter "VehicleJointRuntimeOriginal.CompleteForecastRetainsAllSourceRowsAndChecksCapsBeforeAllocation")
  else()
    set(filter "VehicleJointRuntimeOriginal.ActualSeventhParticipantVirginReadbackAndExactRetry")
  endif()
  add_test(NAME vehicle_runtime_${gate} COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/tied_classification/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
    "$<TARGET_FILE:robo_dyna_vehicle_physical_startup_check>" "${filter}")
  set_tests_properties(vehicle_runtime_${gate} PROPERTIES TIMEOUT 600 RUN_SERIAL TRUE PROCESSORS 2
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
endforeach()
