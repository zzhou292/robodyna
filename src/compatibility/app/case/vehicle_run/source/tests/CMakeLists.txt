cmake_minimum_required(VERSION 3.22)
project(RoboDynaOriginalSourceIO LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Explicit neutral TL source dependency")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
set(ROBO_DYNA_FULL_SHELL_RECORD_TESTS OFF CACHE BOOL "No unrelated fixture tests")
set(ROBO_DYNA_SOURCE_MAPPING_TESTS OFF CACHE BOOL "No unrelated source fixtures")
include("${CMAKE_CURRENT_LIST_DIR}/../OriginalSourceIO.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/CheckProduction.cmake")
option(ROBO_ORIGINAL_SOURCE_IO_TESTS "Build bounded original reader and extraction checks" ON)
if(ROBO_ORIGINAL_SOURCE_IO_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(oracle "${CMAKE_CURRENT_BINARY_DIR}/oracle")
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py"
    --output "${oracle}" COMMAND_ERROR_IS_FATAL ANY)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py" "${CMAKE_CURRENT_LIST_DIR}/baseline.json"
    "${CMAKE_CURRENT_LIST_DIR}/../OriginalSourceIO.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../OriginalSources.cpp")
  # Compile the unchanged real caller against the narrower header dependency.
  # No source graph is constructed or reference/runtime code linked here.
  add_library(original_source_constructor_compile OBJECT "${CMAKE_CURRENT_LIST_DIR}/../OriginalSources.cpp")
  target_link_libraries(original_source_constructor_compile PRIVATE robo_dyna_original_source_io)
  target_compile_options(original_source_constructor_compile PRIVATE -fno-fast-math -ffp-contract=off)
  add_executable(original_source_io_check OriginalSourceIOTest.cpp)
  target_include_directories(original_source_io_check PRIVATE "${oracle}")
  target_link_libraries(original_source_io_check PRIVATE robo_dyna_original_source_io GTest::gtest_main)
  target_compile_options(original_source_io_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME original_source_io_values COMMAND original_source_io_check)
  add_test(NAME original_source_io_sources COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
  set_tests_properties(original_source_io_values original_source_io_sources PROPERTIES
    RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 90)
endif()
