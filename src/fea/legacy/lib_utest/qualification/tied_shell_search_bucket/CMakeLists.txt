cmake_minimum_required(VERSION 3.22)
project(TLTiedSearchBucket LANGUAGES CXX Fortran)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/Native.cmake")
add_executable(tied_search_bucket_native_test NativeTest.cpp)
target_link_libraries(tied_search_bucket_native_test PRIVATE tied_search_bucket_native
  tied_search_oracle GTest::gtest_main)
target_compile_options(tied_search_bucket_native_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME tied_search_bucket_native COMMAND tied_search_bucket_native_test)
add_test(NAME tied_search_bucket_source_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/native/verify_sources.py")
set_tests_properties(tied_search_bucket_native PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
