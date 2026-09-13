cmake_minimum_required(VERSION 3.22)
project(TLSelfContactActiveUses LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/SelfContactActiveUseBinding.cmake")
add_executable(self_contact_active_uses_host
  AreaTest.cpp ActivityTest.cpp ClassificationTest.cpp CinTieTest.cpp CapacityTest.cpp)
target_link_libraries(self_contact_active_uses_host PRIVATE
  tl_self_contact_active_uses GTest::gtest_main)
target_compile_options(self_contact_active_uses_host PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME self_contact_active_uses_host COMMAND self_contact_active_uses_host)
set_tests_properties(self_contact_active_uses_host PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 90)
add_test(NAME self_contact_active_uses_source
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
set_tests_properties(self_contact_active_uses_source PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
