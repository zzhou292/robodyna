cmake_minimum_required(VERSION 3.22)
project(TLNativeType25ActivitySource LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(CUDAToolkit REQUIRED)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/collision/RadiossType25ActivitySource.cmake")
include("${TL_ROOT}/lib_src/collision/RadiossType25FixedMainStartup.cmake")
add_executable(type25_activity_source_host PlanTest.cpp
  "${CMAKE_CURRENT_LIST_DIR}/../radioss_type25_runtime/PhysicalMainTest.cpp")
target_link_libraries(type25_activity_source_host PRIVATE tl_radioss_type25_activity_source
  tl_radioss_type25_fixed_main_startup GTest::gtest_main CUDA::cudart)
target_compile_options(type25_activity_source_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME type25_activity_source_host COMMAND type25_activity_source_host)
set_tests_properties(type25_activity_source_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)

add_test(NAME type25_activity_source_proof COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
