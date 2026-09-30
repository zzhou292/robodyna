cmake_minimum_required(VERSION 3.22)
project(RoboDynaMixedInterfaceSource LANGUAGES CXX Fortran)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
get_filename_component(ROBO_MIXED_INTERFACE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include("${CMAKE_CURRENT_LIST_DIR}/../MixedInterfaceSource.cmake")
# Reuse the complete native chain and its source checks. Build only named owning
# targets, and select intended CTests explicitly (transitive targets are unbuilt).
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_interface_surface" interface-reference)
add_executable(mixed_interface_source_values tests/ValuesTest.cpp)
target_link_libraries(mixed_interface_source_values PRIVATE
  robo_dyna_mixed_interface_source type25_interface_oracle GTest::gtest_main)
target_compile_options(mixed_interface_source_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME mixed_interface_source_values COMMAND mixed_interface_source_values)
set_tests_properties(mixed_interface_source_values PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
add_executable(mixed_interface_source_consumer tests/Consumer.cpp)
target_link_libraries(mixed_interface_source_consumer PRIVATE robo_dyna_mixed_interface_source)
add_test(NAME mixed_interface_source_consumer COMMAND mixed_interface_source_consumer)
if(NOT EXISTS "${ROBO_DYNA_REFERENCE_WORKSPACE}/crash-work/deps")
  message(FATAL_ERROR "Source qualification requires the pinned native reference workspace")
endif()
add_test(NAME mixed_interface_source_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py" --workspace "${ROBO_DYNA_REFERENCE_WORKSPACE}"
  --tl-root "${ROBO_DYNA_TL_ROOT}")
option(ROBO_DYNA_MIXED_INTERFACE_ACTUAL_TESTS "Build explicit complete V5 mixed source census" OFF)
if(ROBO_DYNA_MIXED_INTERFACE_ACTUAL_TESTS)
  add_executable(mixed_interface_source_actual tests/ActualTest.cpp
    "${ROBO_MIXED_INTERFACE_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(mixed_interface_source_actual PRIVATE robo_dyna_mixed_interface_source GTest::gtest_main)
  target_compile_options(mixed_interface_source_actual PRIVATE -fno-fast-math -ffp-contract=off)
  # Forecast and source census execute separately under the owning guard.
endif()
