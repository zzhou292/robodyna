cmake_minimum_required(VERSION 3.22)
project(RoboDynaV5CoatedSource LANGUAGES CXX Fortran)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/../CoatedSource.cmake")
get_filename_component(ROBO_COATED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include("${CMAKE_CURRENT_LIST_DIR}/tests/native/Native.cmake")
add_executable(v5_coated_source_values tests/ValuesTest.cpp tests/CoordinatesTest.cpp tests/NativeTest.cpp
  "${ROBO_COATED_ROOT}/modelio/tied_shell/tests/TinyFixture.cpp")
target_link_libraries(v5_coated_source_values PRIVATE robo_dyna_v5_coated_source v5_coated_native_reference GTest::gtest_main)
add_test(NAME v5_coated_source_values COMMAND v5_coated_source_values)
set_tests_properties(v5_coated_source_values PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
# Reuse the unchanged owning tied-source tests and old topology digest suite.
add_subdirectory("${ROBO_COATED_ROOT}/modelio/tied_shell/search_geometry" tied-coordinate-regression)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." topology-digest-regression)
option(ROBO_DYNA_V5_COATED_ACTUAL_TESTS "Build the explicit V5 source assessment driver" OFF)
if(ROBO_DYNA_V5_COATED_ACTUAL_TESTS)
  include("${ROBO_COATED_ROOT}/case/vehicle_startup/joints/VehicleJointModel.cmake")
  include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25MainGeometry.cmake")
  add_executable(v5_coated_source_actual tests/ActualTest.cpp tests/GeometryTest.cpp
    "${ROBO_COATED_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(v5_coated_source_actual PRIVATE robo_dyna_v5_coated_source robo_dyna_vehicle_joint_model tl_radioss_type25_main_geometry GTest::gtest_main)
  # Deliberately no automatic source-sized CTest. A create-only explicit forecast
  # and separate assessment command require the lead's guarded lane admission.
endif()
