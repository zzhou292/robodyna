cmake_minimum_required(VERSION 3.22)
project(RoboDynaPhysicalDynamics LANGUAGES CXX CUDA)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(CUDAToolkit REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalDynamics.cmake")
# Reuse the complete original startup fixture configuration and its provenance.
add_subdirectory("../vehicle_runtime" "startup")
add_executable(robo_dyna_physical_motion_values tests/MotionSummaryTest.cpp
  "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/rigid_assembly_owner/Fixture.cpp")
target_link_libraries(robo_dyna_physical_motion_values PRIVATE robo_dyna_vehicle_physical_dynamics
  tl_tied_cin_attachment GTest::gtest_main)
target_compile_options(robo_dyna_physical_motion_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_physical_motion_values COMMAND robo_dyna_physical_motion_values)
add_executable(robo_dyna_physical_dynamics_original tests/OriginalTest.cpp
  "${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/physical_attachments/tests/Source.cpp")
target_link_libraries(robo_dyna_physical_dynamics_original PRIVATE robo_dyna_vehicle_physical_dynamics GTest::gtest_main)
target_compile_options(robo_dyna_physical_dynamics_original PRIVATE -fno-fast-math -ffp-contract=off)
foreach(gate forecast free_flight)
  if(gate STREQUAL forecast)
    set(filter "VehiclePhysicalDynamicsOriginal.ForecastCompleteHostWorkspaceBeforeAnyAllocation")
  else()
    set(filter "VehiclePhysicalDynamicsOriginal.CompleteFreeFlightDiscardRetryAndSinglePublication")
  endif()
  add_test(NAME vehicle_physical_dynamics_${gate} COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/tied_classification/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
    "$<TARGET_FILE:robo_dyna_physical_dynamics_original>" "${filter}")
  set_tests_properties(vehicle_physical_dynamics_${gate} PROPERTIES TIMEOUT 600 RUN_SERIAL TRUE PROCESSORS 2
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
endforeach()
