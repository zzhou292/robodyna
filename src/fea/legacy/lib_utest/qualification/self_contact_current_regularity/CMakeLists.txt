cmake_minimum_required(VERSION 3.22)
project(TLSelfContactCurrentRegularity LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/SelfContactCurrentRegularity.cmake")

add_executable(self_contact_current_regularity_host
  ActivityReceiptTest.cpp CapacityIdentityTest.cpp FailureTest.cpp
  GeometryTest.cpp ScalingTest.cpp ValuesTest.cpp)
target_link_libraries(self_contact_current_regularity_host PRIVATE
  tl_self_contact_current_regularity GTest::gtest_main)
target_compile_features(self_contact_current_regularity_host PRIVATE cxx_std_17)
target_compile_options(self_contact_current_regularity_host PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME self_contact_current_regularity_host
  COMMAND self_contact_current_regularity_host)

add_test(NAME self_contact_current_regularity_source
  COMMAND "${Python3_EXECUTABLE}" -B
          "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
add_test(NAME self_contact_current_regularity_syntax
  COMMAND "${Python3_EXECUTABLE}" -B
          "${CMAKE_CURRENT_LIST_DIR}/check_syntax.py"
          --compiler "${CMAKE_CXX_COMPILER}"
          --output "${CMAKE_CURRENT_BINARY_DIR}/syntax")
set_tests_properties(
  self_contact_current_regularity_host
  self_contact_current_regularity_source
  self_contact_current_regularity_syntax
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
