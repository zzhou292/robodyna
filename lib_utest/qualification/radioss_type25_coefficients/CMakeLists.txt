cmake_minimum_required(VERSION 3.22)
project(TLRadiossType25Coefficients LANGUAGES CXX Fortran)
enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
option(TYPE25_COEFFICIENTS_CUDA "Build actual coefficient CUDA qualification" OFF)
if(TYPE25_COEFFICIENTS_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  if(NOT CMAKE_CUDA_COMPILER_ID STREQUAL "NVIDIA")
    message(FATAL_ERROR "Selected CUDA qualification requires NVIDIA nvcc")
  endif()
endif()
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "This pinned native oracle qualifies GNU C++ and Fortran")
endif()
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RadiossType25Coefficients.cmake")
set(native "${CMAKE_CURRENT_BINARY_DIR}/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py"
  --output "${native}" RESULT_VARIABLE prepared ERROR_VARIABLE diagnostic)
if(NOT prepared EQUAL 0)
  message(FATAL_ERROR "Pinned coefficient oracle preparation failed: ${diagnostic}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  native/prepare.py native/source-manifest.json native/MainReference.F.in native/NodalReference.F.in
  native/PairReference.F.in native/original/i25sti3.F native/original/asstifi.F
  native/original/i25stslav.F native/original/i25cor3.F native/original/coefficient-controls.json
  ../radioss_type25_normal/native/prepare.py ../radioss_type25_normal/native/source-manifest.json
  ../radioss_type25_normal/native/original/constant_mod.F)
add_library(type25_coefficient_oracle STATIC NativeOracle.cpp "${native}/MainReference.F"
  "${native}/NodalReference.F" "${native}/PairReference.F")
target_link_libraries(type25_coefficient_oracle PUBLIC tl_radioss_type25_coefficients)
target_compile_options(type25_coefficient_oracle PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-ffixed-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
add_executable(type25_coefficients_host HostTest.cpp UnitsTest.cpp)
target_link_libraries(type25_coefficients_host PRIVATE type25_coefficient_oracle GTest::gtest_main)
add_test(NAME type25_coefficients_host COMMAND type25_coefficients_host)
add_executable(type25_coefficients_consumer HeaderTest.cpp)
# No private precision flags, native oracle, CUDA runtime, or contact owner.
target_link_libraries(type25_coefficients_consumer PRIVATE tl_radioss_type25_coefficients GTest::gtest_main)
add_test(NAME type25_coefficients_consumer COMMAND type25_coefficients_consumer)
add_test(NAME type25_coefficients_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py" --output "${native}" --check)
set_tests_properties(type25_coefficients_host type25_coefficients_consumer type25_coefficients_source
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 90)
if(TYPE25_COEFFICIENTS_CUDA)
  add_executable(type25_coefficients_cuda CudaTest.cu)
  target_link_libraries(type25_coefficients_cuda PRIVATE type25_coefficient_oracle GTest::gtest_main CUDA::cudart)
  set_target_properties(type25_coefficients_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  add_test(NAME type25_coefficients_cuda COMMAND type25_coefficients_cuda)
  set_tests_properties(type25_coefficients_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
endif()
