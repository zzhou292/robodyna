cmake_minimum_required(VERSION 3.18)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RoboDynaTiedAuxiliaryConstraints LANGUAGES CXX)
  find_package(Chrono REQUIRED CONFIG)
  enable_testing()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../TiedAuxiliaryConstraints.cmake")
find_package(GTest REQUIRED)
add_executable(robo_dyna_tied_auxiliary_check tests/TinyFixture.cpp tests/ValueTest.cpp
  tests/ActualSourceTest.cpp ../tests/TinyFixture.cpp)
target_link_libraries(robo_dyna_tied_auxiliary_check PRIVATE robo_dyna_tied_auxiliary_constraints GTest::gtest_main)
target_compile_options(robo_dyna_tied_auxiliary_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME tied_auxiliary_values COMMAND robo_dyna_tied_auxiliary_check --gtest_filter=TiedAuxiliaryValues.*)
set(ROBO_DYNA_TIED_CANONICAL "" CACHE PATH "Explicit original canonical directory")
set(ROBO_DYNA_TIED_SCOPE "" CACHE FILEPATH "Explicit original scope10")
if(ROBO_DYNA_TIED_CANONICAL AND ROBO_DYNA_TIED_SCOPE)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME tied_auxiliary_actual COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/tests/actual_fixture.py"
    "${ROBO_DYNA_TIED_CANONICAL}" "${ROBO_DYNA_TIED_SCOPE}"
    "$<TARGET_FILE:robo_dyna_tied_auxiliary_check>")
endif()
get_property(tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${tests} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
