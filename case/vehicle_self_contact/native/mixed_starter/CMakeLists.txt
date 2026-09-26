cmake_minimum_required(VERSION 3.22)
project(RoboDynaMixedStarterSource LANGUAGES CXX Fortran)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/../MixedStarterSource.cmake")
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_post_gapm" post-gapm-reference)
add_executable(mixed_starter_source_values tests/DigestTest.cpp)
target_link_libraries(mixed_starter_source_values PRIVATE robo_dyna_mixed_starter_source
  tl_radioss_type25_interface_surface type25_post_gapm_oracle type25_current_normals_oracle GTest::gtest_main)
target_compile_options(mixed_starter_source_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME mixed_starter_source_values COMMAND mixed_starter_source_values)
add_executable(mixed_starter_source_consumer tests/Consumer.cpp)
target_link_libraries(mixed_starter_source_consumer PRIVATE robo_dyna_mixed_starter_source)
add_test(NAME mixed_starter_source_consumer COMMAND mixed_starter_source_consumer)
set_tests_properties(mixed_starter_source_values mixed_starter_source_consumer PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)

option(ROBO_DYNA_MIXED_STARTER_ACTUAL "Build explicit complete source/combined-domain Starter gates" OFF)
if(ROBO_DYNA_MIXED_STARTER_ACTUAL)
  get_filename_component(app "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
  include("${app}/case/vehicle_wall/native/WallSource.cmake")
  add_executable(mixed_starter_source_actual tests/ActualTest.cpp
    "${app}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(mixed_starter_source_actual PRIVATE robo_dyna_mixed_starter_source
    robo_dyna_native_envelope_wall_source GTest::gtest_main)
  target_compile_options(mixed_starter_source_actual PRIVATE -fno-fast-math -ffp-contract=off)
  # Forecast and complete construction run separately under the owning guard.
endif()
