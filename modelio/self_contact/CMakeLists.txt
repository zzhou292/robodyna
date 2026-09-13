cmake_minimum_required(VERSION 3.22)
project(RoboDynaOriginalSelfContactSelection LANGUAGES CXX)

find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()

include("${CMAKE_CURRENT_LIST_DIR}/OriginalSelection.cmake")
add_executable(robo_dyna_original_self_contact_selection_check
  tests/SelectionTest.cpp
  "${CMAKE_CURRENT_LIST_DIR}/../tied_shell/tests/TinyFixture.cpp")
target_link_libraries(robo_dyna_original_self_contact_selection_check
  PRIVATE robo_dyna_original_self_contact_selection GTest::gtest_main)
target_compile_options(robo_dyna_original_self_contact_selection_check
  PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME original_self_contact_selection_values
  COMMAND robo_dyna_original_self_contact_selection_check)
add_test(NAME original_self_contact_selection_identity
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")

option(ROBO_DYNA_SELF_CONTACT_ORIGINAL_TESTS
  "Build actual original automatic-single-surface selection gate" OFF)
if(ROBO_DYNA_SELF_CONTACT_ORIGINAL_TESTS)
  foreach(name CANONICAL SCOPE)
    if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
      message(FATAL_ERROR "Original self-contact gate requires ${name}")
    endif()
  endforeach()
  add_executable(robo_dyna_original_self_contact_selection_actual
    tests/OriginalTest.cpp)
  target_link_libraries(robo_dyna_original_self_contact_selection_actual
    PRIVATE robo_dyna_original_self_contact_selection GTest::gtest_main)
  target_compile_options(robo_dyna_original_self_contact_selection_actual
    PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME original_self_contact_selection_actual
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/tests/actual_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}"
      "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_original_self_contact_selection_actual>")
endif()

get_property(selection_tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${selection_tests} PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
