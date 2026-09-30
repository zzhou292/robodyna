cmake_minimum_required(VERSION 3.22)
project(RoboNativeEnvelopeWallSource LANGUAGES CXX)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Explicit qualified TL source")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
include("${CMAKE_CURRENT_LIST_DIR}/WallSource.cmake")
add_executable(native_envelope_wall_values tests/ValuesTest.cpp)
target_link_libraries(native_envelope_wall_values PRIVATE robo_dyna_native_envelope_wall_source GTest::gtest_main)
target_compile_options(native_envelope_wall_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME native_envelope_wall_values COMMAND native_envelope_wall_values)
set_tests_properties(native_envelope_wall_values PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
# Existing source/placement value consumers remain intact; no actual vehicle
# or GPU job is registered implicitly by this first declaration/domain gate.
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/native_spring_ids" spring-id-regression)
add_executable(native_envelope_placement_legacy ../tests/ValuesTest.cpp ../tests/LoadedConfigTest.cpp)
target_link_libraries(native_envelope_placement_legacy PRIVATE robo_dyna_vehicle_wall_values GTest::gtest_main)
add_test(NAME native_envelope_placement_legacy COMMAND native_envelope_placement_legacy)
set_tests_properties(native_envelope_placement_legacy PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)

option(ROBO_DYNA_NATIVE_WALL_ACTUAL_TESTS "Build explicit complete V5 wall domain assessment" OFF)
if(ROBO_DYNA_NATIVE_WALL_ACTUAL_TESTS)
  get_filename_component(_wall_app "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
  include("${_wall_app}/case/vehicle_startup/joints/VehicleJointModel.cmake")
  add_executable(native_envelope_wall_actual tests/ActualTest.cpp
    "${_wall_app}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(native_envelope_wall_actual PRIVATE
    robo_dyna_native_envelope_wall_source robo_dyna_vehicle_joint_model GTest::gtest_main)
  target_compile_options(native_envelope_wall_actual PRIVATE -fno-fast-math -ffp-contract=off)
  # Forecast and actual are separate guarded explicit invocations, never CTest.
endif()
