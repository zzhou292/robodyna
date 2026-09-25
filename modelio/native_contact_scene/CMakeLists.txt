cmake_minimum_required(VERSION 3.22)
project(RoboNativeSceneSource LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
include("${CMAKE_CURRENT_LIST_DIR}/MaterialBridge.cmake")
add_executable(native_scene_material_test MaterialBridgeTest.cpp)
target_link_libraries(native_scene_material_test PRIVATE robo_dyna_native_scene_material GTest::gtest_main)
target_compile_options(native_scene_material_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME native_scene_material COMMAND native_scene_material_test)
add_test(NAME native_scene_export COMMAND "${Python3_EXECUTABLE}" -B -m unittest
  benchmarks.native_contact_scene.test_scene benchmarks.native_contact_scene.reference.test_abi)
add_test(NAME native_scene_json_compat COMMAND "${Python3_EXECUTABLE}" -B -m unittest discover
  -s benchmarks/accepted_payloads -p "test_*.py")
set_tests_properties(native_scene_material native_scene_export native_scene_json_compat PROPERTIES
  WORKING_DIRECTORY "${robo_scene_root}" RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
