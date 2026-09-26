cmake_minimum_required(VERSION 3.22)
project(TLType25GapSource LANGUAGES CXX Fortran)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Pinned gap startup comparison qualifies GNU C++ and Fortran")
endif()
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RadiossType25GapSource.cmake")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../radioss_type25_shell_source" shell-regression)
set(native "${CMAKE_CURRENT_BINARY_DIR}/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py"
  --output "${native}" RESULT_VARIABLE prepared ERROR_VARIABLE diagnostic)
if(NOT prepared EQUAL 0)
  message(FATAL_ERROR "Pinned gap source preparation failed: ${diagnostic}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  native/prepare.py native/Gap.F.in native/source-manifest.json
  ../radioss_type25_coefficients/native/original/i25sti3.F
  ../radioss_type25_fixed_main_startup/native/original/i25neigh.F
  ../radioss_type25_normal/native/original/constant_mod.F)
add_library(type25_gap_source_oracle STATIC NativeOracle.cpp "${native}/Gap.F")
target_link_libraries(type25_gap_source_oracle PUBLIC tl_radioss_type25_gap_source)
target_compile_options(type25_gap_source_oracle PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-ffixed-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
add_executable(type25_gap_source_host HostTest.cpp)
target_link_libraries(type25_gap_source_host PRIVATE type25_gap_source_oracle tl_radioss_type25_shell_source GTest::gtest_main)
add_test(NAME type25_gap_source_host COMMAND type25_gap_source_host)
add_executable(type25_gap_source_consumer Consumer.cpp)
target_link_libraries(type25_gap_source_consumer PRIVATE tl_radioss_type25_gap_source)
add_test(NAME type25_gap_source_consumer COMMAND type25_gap_source_consumer)
add_test(NAME type25_gap_source_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py" --output "${native}" --check)
set_tests_properties(type25_gap_source_host type25_gap_source_consumer type25_gap_source_source
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
