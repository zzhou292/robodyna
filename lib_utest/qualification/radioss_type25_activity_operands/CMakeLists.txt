cmake_minimum_required(VERSION 3.22)
project(TLNativeContactActivityOperands LANGUAGES CXX C CUDA Fortran)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(PHYSICAL_PUBLICATION_CUDA ON CACHE BOOL "Reuse physical publication dependencies" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../physical_publication" physical-publication)
include("${TL_ROOT}/lib_src/collision/RadiossType25ActivityOperands.cmake")
include("${TL_ROOT}/lib_src/collision/RadiossType25FixedMainStartup.cmake")
set(TYPE25_ACTIVITY_NATIVE_TEST_ROOT "" CACHE PATH "Qualified native activity/discovery test source")
if(NOT EXISTS "${TYPE25_ACTIVITY_NATIVE_TEST_ROOT}/NativeOracle.h")
  message(FATAL_ERROR "Provide the qualified native activity/discovery test source")
endif()
set(TYPE25_ACTIVITY_NATIVE_LIFECYCLE_TESTS OFF CACHE BOOL "Separate existing lifecycle gate" FORCE)
add_subdirectory("${TYPE25_ACTIVITY_NATIVE_TEST_ROOT}" native-activity-reference)
# Only qualification binaries link this exact, serial native MSR prefix.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(main_role_native "${CMAKE_CURRENT_BINARY_DIR}/main-role-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/main_node_mask/prepare.py"
  --source "${TYPE25_ACTIVITY_NATIVE_SOURCE_ROOT}" --output "${main_role_native}"
  COMMAND_ERROR_IS_FATAL ANY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  main_node_mask/prepare.py main_node_mask/source.json)
add_library(type25_main_role_native STATIC "${main_role_native}/MainRole.F")
target_compile_options(type25_main_role_native PRIVATE -ffixed-line-length-none -fcheck=all
  -fbacktrace -fno-fast-math -ffp-contract=off)
add_test(NAME type25_main_role_native_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/main_node_mask/prepare.py"
  --source "${TYPE25_ACTIVITY_NATIVE_SOURCE_ROOT}" --output "${main_role_native}" --check)
add_executable(type25_activity_operands_host ValueTest.cpp main_node_mask/HostTest.cpp)
target_link_libraries(type25_activity_operands_host PRIVATE tl_radioss_type25_activity_operands type25_activity_native type25_main_role_native GTest::gtest_main)
target_compile_options(type25_activity_operands_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME type25_activity_operands_host COMMAND type25_activity_operands_host)
add_executable(type25_activity_operands_cuda CudaTest.cu CudaValues.cu main_node_mask/CudaTest.cu)
target_link_libraries(type25_activity_operands_cuda PRIVATE tl_radioss_type25_activity_operands
  tl_radioss_type25_fixed_main_startup type25_activity_native type25_main_role_native GTest::gtest_main CUDA::cudart)
set_target_properties(type25_activity_operands_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(type25_activity_operands_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME type25_activity_operands_cuda COMMAND type25_activity_operands_cuda)
set_tests_properties(type25_activity_operands_host type25_activity_operands_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
