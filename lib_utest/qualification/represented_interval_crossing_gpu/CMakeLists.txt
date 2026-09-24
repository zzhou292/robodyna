cmake_minimum_required(VERSION 3.22)
project(TLRepresentedIntervalCrossingGpu LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RepresentedIntervalCrossingGpu.cmake")
add_executable(represented_interval_crossing_gpu_host
  HostTest.cpp CapacityHostTest.cpp NumericCohortHostTest.cpp NumericCohortCases.h Cases.h)
target_link_libraries(represented_interval_crossing_gpu_host PRIVATE
  tl_represented_interval_crossing_gpu GTest::gtest_main)
target_compile_features(represented_interval_crossing_gpu_host PRIVATE cxx_std_17)
target_compile_options(represented_interval_crossing_gpu_host PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(represented_interval_crossing_gpu_cuda
  GpuTest.cpp GpuFixture.h CapacityCudaTest.cpp NumericCohortCudaTest.cpp
  NumericCohortCases.h CopyFault.cpp CopyFault.h Cases.h)
target_link_libraries(represented_interval_crossing_gpu_cuda PRIVATE
  tl_represented_interval_crossing_gpu CUDA::cudart GTest::gtest_main)
target_compile_features(represented_interval_crossing_gpu_cuda PRIVATE cxx_std_17)
target_compile_options(represented_interval_crossing_gpu_cuda PRIVATE -fno-fast-math -ffp-contract=off)
target_link_options(represented_interval_crossing_gpu_cuda PRIVATE -Wl,--wrap=cudaMemcpyAsync)
add_test(NAME represented_interval_crossing_gpu_host COMMAND represented_interval_crossing_gpu_host)
add_test(NAME represented_interval_crossing_gpu_cuda COMMAND represented_interval_crossing_gpu_cuda)
add_test(NAME represented_interval_crossing_gpu_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(represented_interval_crossing_gpu_host represented_interval_crossing_gpu_source
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 90)
set_tests_properties(represented_interval_crossing_gpu_cuda PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 2 TIMEOUT 180 LABELS "cuda;differential;standalone-native")

add_executable(represented_interval_crossing_gpu_resources Resources.cpp)
target_link_libraries(represented_interval_crossing_gpu_resources PRIVATE
  tl_represented_interval_crossing_gpu CUDA::cudart)
target_compile_features(represented_interval_crossing_gpu_resources PRIVATE cxx_std_17)
include("${CMAKE_CURRENT_LIST_DIR}/Compound.cmake")
add_test(NAME represented_interval_crossing_gpu_capacity_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/verify_capacity_sources.py")
set_tests_properties(represented_interval_crossing_gpu_capacity_source PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30 LABELS "unit;source;cuda-native-capacity")
add_test(NAME represented_interval_crossing_gpu_cohort_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/verify_cohort_sources.py")
set_tests_properties(represented_interval_crossing_gpu_cohort_source PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30 LABELS "unit;source;cuda-native-cohort")

# Optional maintained corpus; ordinary qualification has no external fixture dependency.
if(NATIVE_BENCHMARK_ROOT)
  add_subdirectory(benchmark)
endif()
