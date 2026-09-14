cmake_minimum_required(VERSION 3.22)
project(TLSelfContactPhysicalActivity LANGUAGES CXX C)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(CUDAToolkit REQUIRED)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

add_executable(self_contact_physical_activity_values
  ValueTest.cpp
  "${TL_ROOT}/lib_src/collision/self_contact_physical_activity/Layout.cpp"
  "${TL_ROOT}/lib_src/collision/self_contact_physical_activity/Values.cpp")
target_include_directories(self_contact_physical_activity_values PRIVATE
  "${TL_ROOT}" "${CUDAToolkit_INCLUDE_DIRS}" "/usr/include/eigen3")
target_link_libraries(self_contact_physical_activity_values PRIVATE
  GTest::gtest_main)
target_compile_features(self_contact_physical_activity_values PRIVATE
  cxx_std_17)
target_compile_options(self_contact_physical_activity_values PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME self_contact_physical_activity_values
  COMMAND self_contact_physical_activity_values)

add_test(NAME self_contact_physical_activity_source
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
add_test(NAME self_contact_physical_activity_syntax
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/check_syntax.py"
    --compiler "${CMAKE_CXX_COMPILER}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/syntax")
set_tests_properties(
  self_contact_physical_activity_values
  self_contact_physical_activity_source
  self_contact_physical_activity_syntax
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)

option(SELF_CONTACT_PHYSICAL_ACTIVITY_CUDA
  "Build actual physical owner activity authority gate" OFF)
if(SELF_CONTACT_PHYSICAL_ACTIVITY_CUDA)
  enable_language(CUDA)
  set(PHYSICAL_PUBLICATION_CUDA ON CACHE BOOL
    "Reuse complete physical owner fixture" FORCE)
  add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication"
    physical-publication)
  include(
    "${TL_ROOT}/lib_src/collision/SelfContactPhysicalActivity.cmake")
  add_executable(self_contact_physical_activity_cuda
    CudaTest.cu
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerStartup.cu"
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerAttempt.cu"
    "${CMAKE_CURRENT_LIST_DIR}/../qbat_resident/ReadFault.cu")
  target_link_libraries(self_contact_physical_activity_cuda PRIVATE
    physical_publication_fixture
    tl_self_contact_physical_activity
    CUDA::cudart GTest::gtest_main)
  target_link_options(self_contact_physical_activity_cuda PRIVATE
    "LINKER:--wrap=cudaMemcpyAsync")
  set_target_properties(self_contact_physical_activity_cuda PROPERTIES
    CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(self_contact_physical_activity_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME self_contact_physical_activity_cuda
    COMMAND self_contact_physical_activity_cuda)
  set_tests_properties(self_contact_physical_activity_cuda PROPERTIES
    RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
endif()
