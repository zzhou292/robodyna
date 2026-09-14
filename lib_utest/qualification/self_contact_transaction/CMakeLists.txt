cmake_minimum_required(VERSION 3.22)
project(TLSelfContactTransactionValues LANGUAGES CXX C)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(CUDAToolkit REQUIRED)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

include("${TL_ROOT}/lib_src/collision/FixedTriangleFeatureDiscovery.cmake")
include("${TL_ROOT}/lib_src/collision/RepresentedIntervalCrossing.cmake")
include("${TL_ROOT}/lib_src/collision/SelfContactFilterCertificates.cmake")
add_executable(self_contact_transaction_values
  ValueTest.cpp
  "${TL_ROOT}/lib_src/collision/self_contact_transaction/Values.cpp"
  "${TL_ROOT}/lib_src/collision/self_contact_transaction/Arena.cpp"
  "${TL_ROOT}/lib_src/collision/self_contact_transaction/Limits.cpp"
  "${TL_ROOT}/lib_src/collision/self_contact_transaction/Streaming.cpp")
target_include_directories(self_contact_transaction_values PRIVATE
  "${TL_ROOT}" "${CUDAToolkit_INCLUDE_DIRS}" "/usr/include/eigen3")
target_link_libraries(self_contact_transaction_values PRIVATE
  tl_fixed_triangle_feature_discovery
  tl_self_contact_filter_certificates GTest::gtest_main)
target_compile_features(self_contact_transaction_values PRIVATE cxx_std_17)
target_compile_options(self_contact_transaction_values PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME self_contact_transaction_values
  COMMAND self_contact_transaction_values)

add_executable(self_contact_transaction_medium_coupon
  MediumCouponTest.cpp
  "${TL_ROOT}/lib_src/collision/self_contact_transaction/Values.cpp"
  "${TL_ROOT}/lib_src/collision/self_contact_transaction/Arena.cpp")
target_include_directories(self_contact_transaction_medium_coupon PRIVATE
  "${TL_ROOT}" "${CUDAToolkit_INCLUDE_DIRS}" "/usr/include/eigen3")
target_link_libraries(self_contact_transaction_medium_coupon PRIVATE
  tl_fixed_triangle_feature_discovery
  tl_represented_interval_crossing
  tl_self_contact_filter_certificates GTest::gtest_main)
target_compile_features(
  self_contact_transaction_medium_coupon PRIVATE cxx_std_17)
target_compile_options(self_contact_transaction_medium_coupon PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME self_contact_transaction_medium_coupon
  COMMAND self_contact_transaction_medium_coupon)
set_tests_properties(self_contact_transaction_medium_coupon PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30
  LABELS "coupon;m2;self-contact-medium;prism-axis-certificate")

add_test(NAME self_contact_transaction_source
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
add_test(NAME self_contact_transaction_syntax
  COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/check_syntax.py"
    --compiler "${CMAKE_CXX_COMPILER}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/syntax")
set_tests_properties(
  self_contact_transaction_values
  self_contact_transaction_source
  self_contact_transaction_syntax
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120
  LABELS "unit;m2;self-contact-source-shape;symmetric-edge-area")

option(SELF_CONTACT_TRANSACTION_CUDA
  "Build actual owner/publication self-contact transaction gate" OFF)
if(SELF_CONTACT_TRANSACTION_CUDA)
  enable_language(CUDA)
  set(PHYSICAL_PUBLICATION_CUDA ON CACHE BOOL
    "Reuse complete physical owner fixture" FORCE)
  add_subdirectory(
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication"
    physical-publication)
  include("${TL_ROOT}/lib_src/collision/SelfContactTransaction.cmake")
  add_executable(self_contact_transaction_cuda
    CudaTest.cu
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerStartup.cu"
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerAttempt.cu"
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerSnapshot.cu")
  target_link_libraries(self_contact_transaction_cuda PRIVATE
    physical_publication_fixture
    tl_self_contact_transaction
    CUDA::cudart GTest::gtest_main)
  set_target_properties(self_contact_transaction_cuda PROPERTIES
    CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(self_contact_transaction_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME self_contact_transaction_cuda
    COMMAND self_contact_transaction_cuda)
  set_tests_properties(self_contact_transaction_cuda PROPERTIES
    RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300
    LABELS "coupon;m2;rigid-cin-response")
endif()
