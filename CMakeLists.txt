cmake_minimum_required(VERSION 3.18)
project(CrashApplicationTools LANGUAGES NONE)
find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)
enable_testing()
add_test(NAME yaris_wall_import
  COMMAND "${Python3_EXECUTABLE}" -m unittest discover
          -s "${CMAKE_CURRENT_SOURCE_DIR}/tests" -p "test_yaris_wall.py" -v)
set_tests_properties(yaris_wall_import PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
