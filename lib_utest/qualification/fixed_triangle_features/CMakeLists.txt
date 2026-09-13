cmake_minimum_required(VERSION 3.22)
project(TLFixedTriangleFeatures LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/FixedTriangleFeatureDiscovery.cmake")
add_executable(fixed_triangle_features_host
  AdapterTest.cpp
  CapacityTest.cpp
  ExhaustiveTest.cpp
  GeometryTest.cpp
  IntersectionTest.cpp
  OracleTest.cpp
  StratumTest.cpp)
target_link_libraries(fixed_triangle_features_host PRIVATE
  tl_fixed_triangle_feature_discovery GTest::gtest_main)
target_compile_options(fixed_triangle_features_host PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME fixed_triangle_features_host
  COMMAND fixed_triangle_features_host)
add_test(NAME fixed_triangle_features_source
  COMMAND "${Python3_EXECUTABLE}" -B
          "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
add_test(NAME fixed_triangle_features_shape
  COMMAND "${Python3_EXECUTABLE}" -B
          "${CMAKE_CURRENT_LIST_DIR}/check_syntax.py"
          --compiler "${CMAKE_CXX_COMPILER}"
          --output "${CMAKE_CURRENT_BINARY_DIR}/shape")
set_tests_properties(
  fixed_triangle_features_host
  fixed_triangle_features_source
  fixed_triangle_features_shape
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
