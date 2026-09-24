cmake_minimum_required(VERSION 3.22)
project(TLFixedTriangleFeatures LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/FixedTriangleFeatureDiscovery.cmake")
add_executable(fixed_triangle_features_host
  DiagnosticsTest.cpp
  exact_storage/DiscoverySerialization.h
  AdapterTest.cpp
  CapacityTest.cpp
  ExhaustiveTest.cpp
  ExactStorageTest.cpp
  ExactStorageOracle.h
  GeometryTest.cpp
  IntersectionTest.cpp
  MaskTest.cpp
  OracleTest.cpp
  ParallelTest.cpp
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

# The production wrapper is always adaptive. A separate qualification binary
# substitutes only its four exact predicate entry points; geometry/discovery
# source, compilation flags and publication machinery are identical.
add_library(fixed_triangle_exact_storage_wide STATIC
  exact_storage/WideAdapter.cpp
  "${TL_ROOT}/lib_src/collision/fixed_triangle_features/Geometry.cpp"
  "${TL_ROOT}/lib_src/collision/fixed_triangle_features/Discovery.cpp")
target_link_libraries(fixed_triangle_exact_storage_wide PUBLIC tl_fixed_contact_facets Threads::Threads)
target_compile_features(fixed_triangle_exact_storage_wide PUBLIC cxx_std_17)
target_compile_options(fixed_triangle_exact_storage_wide PRIVATE -fno-fast-math -ffp-contract=off)
foreach(mode IN ITEMS adaptive wide)
  add_executable(exact_storage_${mode}_driver exact_storage/DiscoveryParityDriver.cpp)
  target_compile_options(exact_storage_${mode}_driver PRIVATE -fno-fast-math -ffp-contract=off)
endforeach()
target_link_libraries(exact_storage_adaptive_driver PRIVATE tl_fixed_triangle_feature_discovery)
target_link_libraries(exact_storage_wide_driver PRIVATE fixed_triangle_exact_storage_wide)
add_test(NAME fixed_triangle_exact_storage_discovery_parity
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/exact_storage/compare_discovery.py"
    --adaptive "$<TARGET_FILE:exact_storage_adaptive_driver>"
    --wide "$<TARGET_FILE:exact_storage_wide_driver>"
    --output-dir "${CMAKE_CURRENT_BINARY_DIR}/exact-storage-parity")
set_tests_properties(fixed_triangle_exact_storage_discovery_parity PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 4 TIMEOUT 90 LABELS "unit;host;exact-storage;differential")
