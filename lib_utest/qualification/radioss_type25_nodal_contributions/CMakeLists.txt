cmake_minimum_required(VERSION 3.22)
project(TLNativeNodalContributions LANGUAGES CXX Fortran)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
option(TYPE25_CONTRIBUTIONS_CUDA "Build actual native contribution CUDA qualification" OFF)
if(TYPE25_CONTRIBUTIONS_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()
set(TYPE25_COEFFICIENTS_CUDA ${TYPE25_CONTRIBUTIONS_CUDA})
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../radioss_type25_coefficients" coefficient-regression)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RadiossType25Coefficients.cmake")
set(native "${CMAKE_CURRENT_BINARY_DIR}/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py" --output "${native}"
  RESULT_VARIABLE prepared ERROR_VARIABLE diagnostic)
if(NOT prepared EQUAL 0)
  message(FATAL_ERROR "Pinned contribution source preparation failed: ${diagnostic}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  native/prepare.py native/source-manifest.json native/Boundary.F90 native/BulkWrapper.F90 native/Spring.F.in native/Length.F.in
  native/original/sbulk3.F native/original/rinit3.F
  ../radioss_type25_selection/native/Sources.py ../radioss_type25_normal/native/original/constant_mod.F
  ../radioss_type25_fixed_main_startup/native/original/starter_mvsiz_p.inc)
add_library(type25_contribution_oracle STATIC NativeOracle.cpp
  "${native}/Constants.F90" "${native}/Boundary.F90" "${native}/Bulk.F" "${native}/BulkWrapper.F90"
  "${native}/Spring.F" "${native}/Length.F")
set_target_properties(type25_contribution_oracle PROPERTIES Fortran_MODULE_DIRECTORY "${native}")
target_include_directories(type25_contribution_oracle PRIVATE "${native}")
target_link_libraries(type25_contribution_oracle PUBLIC tl_radioss_type25_coefficients)
target_compile_options(type25_contribution_oracle PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
add_executable(type25_contributions_host HostTest.cpp)
target_link_libraries(type25_contributions_host PRIVATE type25_contribution_oracle type25_coefficient_oracle GTest::gtest_main)
add_test(NAME type25_contributions_host COMMAND type25_contributions_host)
add_executable(type25_contributions_consumer HeaderTest.cpp)
# No oracle, GTest, CUDA or private precision flags in this production consumer.
target_link_libraries(type25_contributions_consumer PRIVATE tl_radioss_type25_coefficients)
add_test(NAME type25_contributions_consumer COMMAND type25_contributions_consumer)
add_test(NAME type25_contributions_source COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py" --output "${native}" --check)
set_tests_properties(type25_contributions_host type25_contributions_consumer type25_contributions_source
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
if(TYPE25_CONTRIBUTIONS_CUDA)
  add_executable(type25_contributions_cuda CudaTest.cu)
  target_link_libraries(type25_contributions_cuda PRIVATE type25_contribution_oracle GTest::gtest_main CUDA::cudart)
  set_target_properties(type25_contributions_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  add_test(NAME type25_contributions_cuda COMMAND type25_contributions_cuda)
  set_tests_properties(type25_contributions_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
endif()
