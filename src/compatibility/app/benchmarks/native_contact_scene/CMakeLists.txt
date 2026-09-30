cmake_minimum_required(VERSION 3.22)
project(RoboNativeContactScene LANGUAGES NONE)
enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(ROBO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_test(NAME native_contact_scene_export COMMAND "${Python3_EXECUTABLE}" -B -m unittest
  benchmarks.native_contact_scene.test_scene)
add_test(NAME native_contact_scene_abi COMMAND "${Python3_EXECUTABLE}" -B -m unittest
  benchmarks.native_contact_scene.reference.test_abi)
set_tests_properties(native_contact_scene_export native_contact_scene_abi PROPERTIES
  WORKING_DIRECTORY "${ROBO_ROOT}" RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
