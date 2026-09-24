cmake_minimum_required(VERSION 3.22)
project(TLRepresentedIntervalCrossing LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RepresentedIntervalCrossing.cmake")

add_executable(represented_interval_crossing_host
  BatchRosterTest.cpp NormalReuseTest.cpp ResultAssertions.h GeometryTest.cpp InvarianceTest.cpp FailureTest.cpp IdentityTest.cpp
  DeepTest.cpp OracleTest.cpp ParallelTest.cpp TranslationProofTest.cpp Oracle.cpp)
target_link_libraries(represented_interval_crossing_host
  PRIVATE tl_represented_interval_crossing GTest::gtest_main)
target_compile_features(represented_interval_crossing_host PRIVATE cxx_std_17)
target_compile_options(represented_interval_crossing_host
  PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME represented_interval_crossing_host
  COMMAND represented_interval_crossing_host)
set_tests_properties(represented_interval_crossing_host PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

add_test(NAME represented_interval_crossing_identity
  COMMAND "${Python3_EXECUTABLE}" -B
          "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(represented_interval_crossing_identity PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
