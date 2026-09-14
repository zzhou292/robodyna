cmake_minimum_required(VERSION 3.18)
project(RoboDynaVehicleShellExecution LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/VehicleShellExecution.cmake")
add_executable(robo_dyna_vehicle_shell_execution_values_check tests/PackingTest.cpp tests/ResourceTest.cpp)
target_link_libraries(robo_dyna_vehicle_shell_execution_values_check PRIVATE
  robo_dyna_vehicle_shell_execution_values GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_shell_execution_values_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_shell_execution_values COMMAND robo_dyna_vehicle_shell_execution_values_check)
set_tests_properties(vehicle_shell_execution_values PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)

add_executable(robo_dyna_vehicle_shell_execution_small_check tests/SmallCompositionTest.cpp)
target_link_libraries(robo_dyna_vehicle_shell_execution_small_check PRIVATE
  robo_dyna_vehicle_shell_execution_values tl_shell_physical_binding GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_shell_execution_small_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_shell_execution_small COMMAND robo_dyna_vehicle_shell_execution_small_check)
set_tests_properties(vehicle_shell_execution_small PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)

option(ROBO_DYNA_VEHICLE_EXECUTION_ORIGINAL "Complete authenticated retained original vehicle gate" OFF)
if(ROBO_DYNA_VEHICLE_EXECUTION_ORIGINAL)
  include("${CMAKE_CURRENT_LIST_DIR}/OriginalTests.cmake")
endif()
option(ROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE
  "Register full canonical V5 self-contact runtime acceptance tests" OFF)
option(ROBO_DYNA_VEHICLE_SELF_CONTACT_SOURCE_ORIGINAL "Complete V5 centered-shell self-contact source census" OFF)
if(ROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE AND
   NOT ROBO_DYNA_VEHICLE_SELF_CONTACT_SOURCE_ORIGINAL)
  message(FATAL_ERROR
    "V5 self-contact acceptance requires the original self-contact source fixture")
endif()
if(ROBO_DYNA_VEHICLE_SELF_CONTACT_SOURCE_ORIGINAL)
  if(NOT ROBO_DYNA_VEHICLE_EXECUTION_ORIGINAL)
    message(FATAL_ERROR "Self-contact source census requires the authenticated original fixture")
  endif()
  if(NOT DEFINED CMAKE_CUDA_COMPILER OR
      "${CMAKE_CUDA_COMPILER}" STREQUAL "" OR
      NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR
      "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "")
    message(FATAL_ERROR
      "Self-contact source census requires explicit CMAKE_CUDA_COMPILER and CMAKE_CUDA_ARCHITECTURES")
  endif()
  enable_language(CUDA)
  include("${CMAKE_CURRENT_LIST_DIR}/SelfContactTests.cmake")
endif()
