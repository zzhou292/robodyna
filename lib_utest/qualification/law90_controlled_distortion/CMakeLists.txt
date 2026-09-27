cmake_minimum_required(VERSION 3.22)
project(TLLaw90ControlledDistortion LANGUAGES CXX C Fortran)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(GTest REQUIRED)
enable_testing()
set(TL_DISTORTION_FORCE_CUDA OFF CACHE BOOL "Shared distortion CPU oracle" FORCE)
add_subdirectory(../solid_distortion_force shared_distortion)
# The frozen H24 oracle predates nested embedding and has a root-relative include.
# Supply its actual prepared include directory without changing the frozen oracle.
target_include_directories(solid24_ic1_native PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/shared_distortion/legacy_heph/native/prepared")
set(TL_LAW90_SOLID18_FORCE_NATIVE ON CACHE BOOL "Independent LAW90 force" FORCE)
set(TL_LAW90_SOLID18_FORCE_CUDA OFF CACHE BOOL "No unrelated GPU target" FORCE)
set(TL_LAW90_NATIVE_MODULUS_OBSERVER ON)
add_subdirectory(../law90_solid18_force ordinary_force)
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/native/prepare_sources.py"
  --output "${prepared}" COMMAND_ERROR_IS_FATAL ANY)
add_library(law90_control_parameter_native STATIC native/ParameterProbe.F90)
target_include_directories(law90_control_parameter_native PRIVATE "${prepared}"
  "$<TARGET_PROPERTY:heph_force_native,Fortran_MODULE_DIRECTORY>"
  "$<TARGET_PROPERTY:solid24_ic1_native,Fortran_MODULE_DIRECTORY>")
target_compile_options(law90_control_parameter_native PRIVATE -cpp -ffree-line-length-none -fcheck=bounds
  -fno-fast-math -ffp-contract=off)
target_link_libraries(law90_control_parameter_native PUBLIC distortion_force_probe law90_solid18_force_native)
add_dependencies(law90_control_parameter_native solid24_ic1_native)
add_executable(law90_controlled_distortion_host HostTest.cpp)
target_include_directories(law90_controlled_distortion_host PRIVATE "${TL_ROOT}")
target_compile_features(law90_controlled_distortion_host PRIVATE cxx_std_17)
target_compile_options(law90_controlled_distortion_host PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(law90_controlled_distortion_host PRIVATE law90_control_parameter_native GTest::gtest_main)
add_test(NAME law90_controlled_distortion_host COMMAND law90_controlled_distortion_host)
add_test(NAME law90_controlled_distortion_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/native/prepare_sources.py" --output "${prepared}" --check)

option(TL_LAW90_CONTROL_CUDA "Actual CUDA controlled caller recurrence" OFF)
if(TL_LAW90_CONTROL_CUDA)
  enable_language(CUDA)
  add_executable(law90_controlled_distortion_cuda CudaTest.cu)
  target_include_directories(law90_controlled_distortion_cuda PRIVATE "${TL_ROOT}")
  target_compile_features(law90_controlled_distortion_cuda PRIVATE cuda_std_17)
  target_compile_options(law90_controlled_distortion_cuda PRIVATE --fmad=false --prec-div=true --prec-sqrt=true --ftz=false
    -Xcompiler=-fno-fast-math,-ffp-contract=off)
  target_link_libraries(law90_controlled_distortion_cuda PRIVATE law90_control_parameter_native GTest::gtest_main)
  add_test(NAME law90_controlled_distortion_cuda COMMAND law90_controlled_distortion_cuda)
endif()
get_property(tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${tests} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
