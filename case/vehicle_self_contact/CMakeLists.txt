cmake_minimum_required(VERSION 3.22)
project(RoboDynaVehicleSelfContactValues LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "TL source tree with active-use APIs")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactActiveUseBinding.cmake")
  message(FATAL_ERROR "Vehicle self-contact values require the composed TL active-use tree")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactValues.cmake")
add_executable(robo_dyna_vehicle_self_contact_values_check
  tests/SelectedSourceTest.cpp tests/InitialCensusValuesTest.cpp)
target_link_libraries(robo_dyna_vehicle_self_contact_values_check PRIVATE
  robo_dyna_vehicle_self_contact_values GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_self_contact_values_check PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_self_contact_values
  COMMAND robo_dyna_vehicle_self_contact_values_check)
set_tests_properties(vehicle_self_contact_values PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 90)

add_executable(robo_dyna_vehicle_self_contact_runtime_values_check
  tests/RuntimeValuesTest.cpp RuntimeBudget.cpp)
target_link_libraries(
  robo_dyna_vehicle_self_contact_runtime_values_check PRIVATE
  robo_dyna_vehicle_self_contact_values GTest::gtest_main)
target_compile_options(
  robo_dyna_vehicle_self_contact_runtime_values_check PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_self_contact_runtime_values
  COMMAND robo_dyna_vehicle_self_contact_runtime_values_check)
set_tests_properties(vehicle_self_contact_runtime_values PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 90)

add_test(NAME vehicle_self_contact_runtime_source_proof
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/tests/verify_runtime_sources.py"
    "${CMAKE_CURRENT_LIST_DIR}/../.." "${ROBO_DYNA_TL_ROOT}")
set_tests_properties(vehicle_self_contact_runtime_source_proof PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)

option(ROBO_DYNA_VEHICLE_SELF_CONTACT_RUNTIME
  "Build the opt-in self-only and wall+self runtime factories" OFF)
if(ROBO_DYNA_VEHICLE_SELF_CONTACT_RUNTIME)
  if(NOT DEFINED CMAKE_CUDA_COMPILER OR
      "${CMAKE_CUDA_COMPILER}" STREQUAL "" OR
      NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR
      "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "")
    message(FATAL_ERROR
      "Self-contact runtime requires explicit CUDA compiler and architectures")
  endif()
  enable_language(CUDA)
  include("${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactRuntime.cmake")
endif()
