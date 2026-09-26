cmake_minimum_required(VERSION 3.22)
project(RoboDynaSelectedShellMainSource LANGUAGES CXX Fortran)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
get_filename_component(ROBO_MAIN_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include("${CMAKE_CURRENT_LIST_DIR}/../SelectedShellMainSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/tests/native/Native.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/tests/solid_native/Native.cmake")
# Existing independent scalar oracles and the authenticated context regressions.
set(TYPE25_COEFFICIENTS_CUDA OFF)
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_coated_coefficients" native-coefficients)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../nodal_correction" corrected-source-regression)
add_executable(selected_shell_main_values tests/SupportTest.cpp tests/ContextTest.cpp tests/SupportQueryTest.cpp tests/SolidSupportTest.cpp)
target_link_libraries(selected_shell_main_values PRIVATE selected_main_support_native mixed_solid_support_native coated_coefficient_oracle GTest::gtest_main)
target_compile_options(selected_shell_main_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME selected_shell_main_values COMMAND selected_shell_main_values)
set_tests_properties(selected_shell_main_values selected_main_support_native_source PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
add_executable(selected_shell_main_consumer tests/Consumer.cpp)
target_link_libraries(selected_shell_main_consumer PRIVATE robo_dyna_selected_shell_main_source)
add_test(NAME selected_shell_main_consumer COMMAND selected_shell_main_consumer)
option(ROBO_DYNA_SELECTED_MAIN_ACTUAL_TESTS "Build explicit whole V5 main-source gate" OFF)
if(ROBO_DYNA_SELECTED_MAIN_ACTUAL_TESTS)
  add_executable(selected_shell_main_actual tests/ActualTest.cpp
    "${ROBO_MAIN_SOURCE_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(selected_shell_main_actual PRIVATE robo_dyna_selected_shell_main_source GTest::gtest_main)
  target_compile_options(selected_shell_main_actual PRIVATE -fno-fast-math -ffp-contract=off)
  # Source-sized forecast and one assessment run only under the guarded lane.
endif()

option(ROBO_DYNA_MIXED_SUPPORT_ACTUAL_TESTS "Build explicit complete mixed support-query census" OFF)
if(ROBO_DYNA_MIXED_SUPPORT_ACTUAL_TESTS)
  if(NOT EXISTS "${ROBO_DYNA_REFERENCE_WORKSPACE}/crash-work/deps")
    message(FATAL_ERROR "Post-GAPM source assessment requires the pinned reference workspace")
  endif()
  add_test(NAME post_gapm_controls_source COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/tests/verify_postgapm_controls.py"
    --workspace "${ROBO_DYNA_REFERENCE_WORKSPACE}")
  include("${CMAKE_CURRENT_LIST_DIR}/../MixedInterfaceSource.cmake")
  add_executable(mixed_main_support_actual tests/SupportActualTest.cpp
    "${ROBO_MAIN_SOURCE_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(mixed_main_support_actual PRIVATE robo_dyna_selected_shell_main_source
    robo_dyna_mixed_interface_source GTest::gtest_main)
  target_compile_options(mixed_main_support_actual PRIVATE -fno-fast-math -ffp-contract=off)
  # One explicit bounded source census, never an implicit CTest/full-model run.
endif()
