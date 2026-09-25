cmake_minimum_required(VERSION 3.22)
project(Type25CandidateOracle LANGUAGES CXX Fortran)
enable_testing()
find_package(GTest REQUIRED)
option(TYPE25_CANDIDATE_CUDA "Build actual native candidate CUDA gate" OFF)
if(TYPE25_CANDIDATE_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
add_library(tl_type25_candidate_filter INTERFACE)
target_include_directories(tl_type25_candidate_filter INTERFACE "${TL_ROOT}")
target_compile_features(tl_type25_candidate_filter INTERFACE cxx_std_17)
target_compile_options(tl_type25_candidate_filter INTERFACE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--ftz=false;--prec-div=true;--prec-sqrt=true>")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Pinned native qualification requires GNU C++ and Fortran")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(native "${CMAKE_CURRENT_BINARY_DIR}/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py"
  --output "${native}" RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "Pinned PEN3 preparation failed")
endif()
add_library(pen3_native STATIC "${native}/Constants.F90" "${native}/NativePen3.F" "${native}/Wrapper.F90" "${native}/NativeScreen.F")
target_include_directories(pen3_native PRIVATE "${native}")
target_compile_options(pen3_native PRIVATE -ffixed-line-length-none -ffree-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
add_executable(type25_pen3_cohort_probe Main.cpp)
target_compile_features(type25_pen3_cohort_probe PRIVATE cxx_std_17)
target_compile_options(type25_pen3_cohort_probe PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(type25_pen3_cohort_probe PRIVATE pen3_native)
add_test(NAME type25_pen3_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py" --output "${native}" --check)
# Run the probe directly with its entire stdout retained; the result is an
# experiment, not a pass assertion that all-T3 and mixed membership must agree.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS native/prepare.py native/Wrapper.F90
  native/source-manifest.json native/original/i25pen3.F
  ../radioss_type25_normal/native/original/constant_mod.F)

add_executable(type25_candidate_host HostTest.cpp)
target_link_libraries(type25_candidate_host PRIVATE tl_type25_candidate_filter pen3_native GTest::gtest_main)
add_test(NAME type25_candidate_host COMMAND type25_candidate_host)
if(TYPE25_CANDIDATE_CUDA)
  add_executable(type25_candidate_cuda CudaTest.cu)
  target_link_libraries(type25_candidate_cuda PRIVATE tl_type25_candidate_filter pen3_native GTest::gtest_main CUDA::cudart)
  set_target_properties(type25_candidate_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  add_test(NAME type25_candidate_cuda COMMAND type25_candidate_cuda)
endif()
