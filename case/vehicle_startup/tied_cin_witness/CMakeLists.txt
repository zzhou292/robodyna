cmake_minimum_required(VERSION 3.22)
project(RoboDynaTiedCinWitness LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/../TiedCinWitnessValues.cmake")
add_executable(robo_dyna_tied_cin_witness_values_check tests/ValueTest.cpp)
target_link_libraries(robo_dyna_tied_cin_witness_values_check PRIVATE
  robo_dyna_tied_cin_witness_values GTest::gtest_main)
target_compile_options(robo_dyna_tied_cin_witness_values_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME tied_cin_witness_values COMMAND robo_dyna_tied_cin_witness_values_check)

option(ROBO_DYNA_TIED_CIN_WITNESS_ACTUAL "Original complete shell/witness roster gate" OFF)
if(ROBO_DYNA_TIED_CIN_WITNESS_ACTUAL)
  enable_language(CUDA)
  enable_language(Fortran)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  include("${CMAKE_CURRENT_LIST_DIR}/../TiedCinWitnessRoster.cmake")
  include("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/tied_shell_patch/Native.cmake")
  foreach(name CANONICAL SCOPE DECLARATIONS GLASS_RESOLUTION)
    set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
    if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
      message(FATAL_ERROR "Original CIN witness gate requires ${name}")
    endif()
  endforeach()
  set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Frozen original glass sidecar SHA256")
  string(LENGTH "${ROBO_DYNA_VEHICLE_GLASS_SHA256}" sha_length)
  if(NOT sha_length EQUAL 64)
    message(FATAL_ERROR "Original CIN witness gate requires glass SHA256")
  endif()
  add_executable(robo_dyna_tied_cin_witness_actual_check tests/ActualSourceTest.cpp
    ../tied_cin/tests/ActualSourceTest.cpp ../tied_classification/tests/ActualClassification.cpp)
  target_link_libraries(robo_dyna_tied_cin_witness_actual_check PRIVATE
    robo_dyna_tied_cin_witness_roster tied_patch_oracle GTest::gtest_main)
  target_compile_options(robo_dyna_tied_cin_witness_actual_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME tied_cin_witness_actual COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../tied_classification/tests/actual_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
    "$<TARGET_FILE:robo_dyna_tied_cin_witness_actual_check>" "TiedCinWitnessActual.*")
  set_tests_properties(tied_cin_witness_actual PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300
    ENVIRONMENT "ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256}")
endif()
