cmake_minimum_required(VERSION 3.22)
project(TLShellExecutionBinding LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/assembly/ShellPhysicalBinding.cmake")
add_executable(shell_execution_host CatalogTest.cpp BindingTest.cpp PhysicalTest.cpp)
target_link_libraries(shell_execution_host PRIVATE tl_shell_physical_binding GTest::gtest_main)
target_compile_options(shell_execution_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME shell_execution_host COMMAND shell_execution_host)
set_tests_properties(shell_execution_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

# Existing owner tests, compiled once with this changed catalog/failure layout.
add_executable(shell_execution_legacy
  ../qbat_catalog/ValueTest.cpp ../qbat_catalog/ScopeTest.cpp ../qbat_catalog/FailureTest.cpp
  ../shell_physical_binding/HostTest.cpp)
target_link_libraries(shell_execution_legacy PRIVATE tl_shell_physical_binding GTest::gtest_main)
target_compile_options(shell_execution_legacy PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME shell_execution_legacy COMMAND shell_execution_legacy)
set_tests_properties(shell_execution_legacy PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
