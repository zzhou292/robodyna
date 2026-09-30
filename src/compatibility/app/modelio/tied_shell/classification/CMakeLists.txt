cmake_minimum_required(VERSION 3.22)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RoboDynaTiedClassificationSource LANGUAGES CXX)
  find_package(Chrono REQUIRED CONFIG)
  enable_testing()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../TiedClassificationContext.cmake")
find_package(GTest REQUIRED)
add_executable(robo_dyna_tied_classification_source_check tests/Fixture.cpp tests/ValueTest.cpp
  ../auxiliary/tests/TinyFixture.cpp ../auxiliary/tests/ValueTest.cpp ../tests/TinyFixture.cpp)
target_link_libraries(robo_dyna_tied_classification_source_check PRIVATE
  robo_dyna_tied_classification_context GTest::gtest_main)
add_test(NAME tied_classification_source_values COMMAND robo_dyna_tied_classification_source_check)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME tied_classification_importer_topology COMMAND "${Python3_EXECUTABLE}" -B
  -m unittest tests.test_tied_classification_topology)
set_tests_properties(tied_classification_importer_topology PROPERTIES
  WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/../../.." RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
set_tests_properties(tied_classification_source_values PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
