cmake_minimum_required(VERSION 3.22)
project(RoboDynaBeam18Source LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/Source.cmake")
add_executable(robo_dyna_source_fields_check tests/FieldsTest.cpp)
target_link_libraries(robo_dyna_source_fields_check PRIVATE robo_dyna_beam18_source GTest::gtest_main)
add_test(NAME source_fields_shared COMMAND robo_dyna_source_fields_check)
option(ROBO_DYNA_BEAM18_ORIGINAL_TESTS "Actual142 original beam/source/reference gate" OFF)
if(ROBO_DYNA_BEAM18_ORIGINAL_TESTS)
  foreach(name CANONICAL SCOPE BEAM18_FIXTURE)
    if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
      message(FATAL_ERROR "Explicit original ${name} is required")
    endif()
  endforeach()
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B
    "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/beam18_force/prepare_fixture.py"
    --fixture "${ROBO_DYNA_VEHICLE_BEAM18_FIXTURE}" --output "${CMAKE_CURRENT_BINARY_DIR}/fixture/OriginalFixture.h"
    RESULT_VARIABLE status)
  if(NOT status EQUAL 0)
    message(FATAL_ERROR "Original qualified beam fixture authentication failed")
  endif()
  add_executable(robo_dyna_beam18_source_check tests/OriginalTest.cpp)
  target_include_directories(robo_dyna_beam18_source_check PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/fixture"
    "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/beam18_reference")
  target_link_libraries(robo_dyna_beam18_source_check PRIVATE robo_dyna_beam18_source GTest::gtest_main)
  target_compile_options(robo_dyna_beam18_source_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME beam18_source_original COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_beam18_source_check>" "Beam18SourceOriginal.*")
endif()
# The shared field/curve extraction must preserve existing solid source behavior.
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../solid_source" "solid-source-regression")
get_property(source_tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${source_tests} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
