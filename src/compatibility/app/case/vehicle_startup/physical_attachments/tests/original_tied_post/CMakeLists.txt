cmake_minimum_required(VERSION 3.22)
project(RoboDynaOriginalTiedPost LANGUAGES CXX)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(CUDAToolkit REQUIRED)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../.." ABSOLUTE)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Explicit qualified TL source headers and value producers")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
set(ROBO_DYNA_FULL_SHELL_RECORD_TESTS OFF CACHE BOOL "No unrelated fixture tests")
set(ROBO_DYNA_SOURCE_MAPPING_TESTS OFF CACHE BOOL "No unrelated source fixtures")
# Reuse the predecessor reader/rejection and product-dependency qualification.
add_subdirectory("${APP_ROOT}/case/vehicle_run/source/tests" "source-io")
include("${APP_ROOT}/case/vehicle_startup/TiedSearchPostKinChkValues.cmake")
# Compile the exact production helpers and caller without constructing sources
# or linking the CUDA startup driver into a test executable.
add_library(original_tied_post_callers OBJECT
  "${CMAKE_CURRENT_LIST_DIR}/../../OriginalTiedPost.cpp"
  "${APP_ROOT}/case/vehicle_run/source/OriginalYaris.cpp" CompileContract.cpp)
target_include_directories(original_tied_post_callers PRIVATE
  ${CUDAToolkit_INCLUDE_DIRS} "${ROBO_DYNA_TL_ROOT}/lib_src")
target_link_libraries(original_tied_post_callers PRIVATE
  robo_dyna_original_source_io robo_dyna_tied_post_kinchk_values)
target_compile_options(original_tied_post_callers PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(original_tied_post_values
  "${APP_ROOT}/case/vehicle_startup/tied_classification/tests/ValueTest.cpp"
  "${APP_ROOT}/case/vehicle_startup/tied_post_kinchk/tests/ValueTest.cpp"
  "${APP_ROOT}/modelio/tied_shell/tests/TinyFixture.cpp")
target_link_libraries(original_tied_post_values PRIVATE robo_dyna_tied_post_kinchk_values GTest::gtest_main)
target_compile_options(original_tied_post_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME original_tied_post_values COMMAND original_tied_post_values)
add_test(NAME original_tied_post_sources COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(original_tied_post_values original_tied_post_sources PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
