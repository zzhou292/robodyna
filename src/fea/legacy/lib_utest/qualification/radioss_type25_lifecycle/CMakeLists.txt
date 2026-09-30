cmake_minimum_required(VERSION 3.22)
project(TLRadiossType25Lifecycle LANGUAGES CXX Fortran)
enable_testing()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
option(TYPE25_LIFECYCLE_CUDA "Build actual native lifecycle CUDA qualification" OFF)
if(TYPE25_LIFECYCLE_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()
set(TYPE25_SELECTION_CUDA ${TYPE25_LIFECYCLE_CUDA})
set(TYPE25_COEFFICIENTS_CUDA ${TYPE25_LIFECYCLE_CUDA})
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../radioss_type25_selection" selection)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../radioss_type25_coefficients" coefficients)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RadiossType25Lifecycle.cmake")
set(native "${CMAKE_CURRENT_BINARY_DIR}/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py"
  --output "${native}" RESULT_VARIABLE prepared ERROR_VARIABLE diagnostic)
if(NOT prepared EQUAL 0)
  message(FATAL_ERROR "Pinned lifecycle oracle preparation failed: ${diagnostic}")
endif()
file(GLOB native_inputs CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/native/*"
  "${CMAKE_CURRENT_LIST_DIR}/native/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${native_inputs})
add_library(type25_lifecycle_oracle STATIC NativeOracle.cpp
  native/LifecycleBegin.cpp native/LifecycleOptimized.cpp native/LifecycleSliding.cpp native/LifecycleKeep.cpp
  "${native}/LifecycleConstants.F90" "${native}/LifecycleBoundary.F90"
  "${native}/LifecycleMemory.F90" "${native}/LifecycleObservation.F90"
  "${native}/LifecycleOptimized.F90" "${native}/LifecycleSliding.F90"
  "${native}/LifecycleMembership.F" "${native}/LifecyclePhases.F"
  "${native}/OptimizedCandidates.F" "${native}/PrepSliding1.F"
  "${native}/PrepSliding2.F" "${native}/KeepContact.F")
file(MAKE_DIRECTORY "${native}/modules")
set_target_properties(type25_lifecycle_oracle PROPERTIES Fortran_MODULE_DIRECTORY "${native}/modules")
target_include_directories(type25_lifecycle_oracle PRIVATE "${native}" "${native}/modules")
target_link_libraries(type25_lifecycle_oracle PUBLIC tl_radioss_type25_lifecycle
  type25_selection_oracle type25_coefficient_oracle type25_geometry_oracle type25_friction_oracle)
target_compile_options(type25_lifecycle_oracle PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-ffixed-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
add_executable(type25_lifecycle_host HostTest.cpp PhaseBoundaryTest.cpp NormalViewHostTest.cpp)
target_link_libraries(type25_lifecycle_host PRIVATE type25_lifecycle_oracle GTest::gtest_main)
add_test(NAME type25_lifecycle_host COMMAND type25_lifecycle_host)
add_executable(type25_lifecycle_consumer ConsumerTest.cpp NormalViewConsumer.cpp)
target_link_libraries(type25_lifecycle_consumer PRIVATE tl_radioss_type25_lifecycle GTest::gtest_main)
add_test(NAME type25_lifecycle_consumer COMMAND type25_lifecycle_consumer)
add_test(NAME type25_lifecycle_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/native/prepare.py" --output "${native}" --check)
set_tests_properties(type25_lifecycle_host type25_lifecycle_consumer type25_lifecycle_source
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
if(TYPE25_LIFECYCLE_CUDA)
  # Parent selection owns precision flags and NVIDIA compiler admission.
  add_executable(type25_lifecycle_cuda CudaTest.cu)
  target_link_libraries(type25_lifecycle_cuda PRIVATE type25_lifecycle_oracle GTest::gtest_main CUDA::cudart)
  set_target_properties(type25_lifecycle_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  add_test(NAME type25_lifecycle_cuda COMMAND type25_lifecycle_cuda)
  set_tests_properties(type25_lifecycle_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
  # Separate TU/executable: the reused fixture declares its private CUDA kernels.
  add_executable(type25_lifecycle_normal_view_cuda NormalViewCudaTest.cu)
  target_link_libraries(type25_lifecycle_normal_view_cuda PRIVATE type25_lifecycle_oracle GTest::gtest_main CUDA::cudart)
  set_target_properties(type25_lifecycle_normal_view_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  add_test(NAME type25_lifecycle_normal_view_cuda COMMAND type25_lifecycle_normal_view_cuda)
  set_tests_properties(type25_lifecycle_normal_view_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
endif()
