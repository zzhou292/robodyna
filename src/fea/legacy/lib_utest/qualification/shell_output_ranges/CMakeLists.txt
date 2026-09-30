cmake_minimum_required(VERSION 3.22)
project(TLShellOutputRanges LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/elements/ShellPhysicalOwnerValues.cmake")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py"
  COMMAND_ERROR_IS_FATAL ANY)
add_executable(shell_output_ranges_host EnvelopeTest.cpp BindingTest.cpp ScopeTest.cpp ExecutionTest.cpp
  oracle/FrozenExecution.cpp)
target_include_directories(shell_output_ranges_host PRIVATE "${CMAKE_CURRENT_LIST_DIR}/oracle")
target_link_libraries(shell_output_ranges_host PRIVATE tl_shell_physical_owner_values GTest::gtest_main)
target_compile_options(shell_output_ranges_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME shell_output_ranges_host COMMAND shell_output_ranges_host)
add_test(NAME shell_output_ranges_source COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(shell_output_ranges_host shell_output_ranges_source PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
