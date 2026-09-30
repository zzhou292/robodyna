cmake_minimum_required(VERSION 3.22)
project(RoboDynaInitialSurfaceSource LANGUAGES CXX Fortran)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
get_filename_component(ROBO_INITIAL_SURFACE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include("${CMAKE_CURRENT_LIST_DIR}/../InitialSurfaceSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_surface_source/native/Native.cmake")
add_executable(initial_surface_source_values
  tests/CertificateTest.cpp tests/ContextTest.cpp tests/DigestTest.cpp tests/NativeTest.cpp)
target_link_libraries(initial_surface_source_values PRIVATE
  robo_dyna_initial_surface_source type25_surface_source_native GTest::gtest_main)
target_compile_options(initial_surface_source_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME initial_surface_source_values COMMAND initial_surface_source_values)
set_tests_properties(initial_surface_source_values PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
add_executable(initial_surface_source_consumer tests/Consumer.cpp)
target_link_libraries(initial_surface_source_consumer PRIVATE robo_dyna_initial_surface_source)
add_test(NAME initial_surface_source_consumer COMMAND initial_surface_source_consumer)
# Existing selection/parser controls remain owning regressions; no source-sized
# tests run implicitly through this subdirectory.
add_subdirectory("${ROBO_INITIAL_SURFACE_ROOT}/modelio/self_contact" original-selection-regression)
if(NOT EXISTS "${ROBO_DYNA_REFERENCE_WORKSPACE}/crash-work/deps")
  message(FATAL_ERROR "Source qualification requires the pinned native reference workspace")
endif()
add_test(NAME initial_surface_source_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py" --workspace "${ROBO_DYNA_REFERENCE_WORKSPACE}")
option(ROBO_DYNA_INITIAL_SURFACE_ACTUAL_TESTS "Build explicit full V5 initial surface census" OFF)
if(ROBO_DYNA_INITIAL_SURFACE_ACTUAL_TESTS)
  add_executable(initial_surface_source_actual tests/ActualTest.cpp
    "${ROBO_INITIAL_SURFACE_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(initial_surface_source_actual PRIVATE robo_dyna_initial_surface_source GTest::gtest_main)
  target_compile_options(initial_surface_source_actual PRIVATE -fno-fast-math -ffp-contract=off)
  # Separate forecast and create-only source commands belong to the guarded lane.
endif()
