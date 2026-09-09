cmake_minimum_required(VERSION 3.18)
project(CrashApplicationTools LANGUAGES NONE)
find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)
enable_testing()
add_test(NAME yaris_wall_import
  COMMAND "${Python3_EXECUTABLE}" -m unittest discover
          -s "${CMAKE_CURRENT_SOURCE_DIR}/tests" -p "test_yaris_wall.py" -v)
set_tests_properties(yaris_wall_import PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)

option(CRASH_ENABLE_CHRONO_CHECK "Build the external Chrono wall integration check" OFF)
option(CRASH_ENABLE_CHRONO_SNAPSHOT_CHECK "Build the TL host-view to Chrono accepted-surface check" OFF)
option(CRASH_ENABLE_TL_CHRONO_CHECK "Build the CUDA shell qualification to Chrono accepted-state test" OFF)
if(CRASH_ENABLE_CHRONO_CHECK OR CRASH_ENABLE_CHRONO_SNAPSHOT_CHECK OR CRASH_ENABLE_TL_CHRONO_CHECK)
  add_subdirectory(chrono)
endif()

add_test(NAME vehicle_geometry_unit
  COMMAND "${Python3_EXECUTABLE}" -B -m unittest discover
          -s "${CMAKE_CURRENT_SOURCE_DIR}/tests" -p "test_vehicle_geometry.py" -v)
set_tests_properties(vehicle_geometry_unit PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)

# Full-model data stays outside git. Supplying its prepared path registers an
# explicit integration gate; missing assets fail rather than produce a skip.
set(CRASH_YARIS_VEHICLE_ASSETS "" CACHE PATH "Prepared SHA-pinned Yaris vehicle geometry")
if(CRASH_YARIS_VEHICLE_ASSETS)
  add_test(NAME yaris_vehicle_artifacts
    COMMAND "${Python3_EXECUTABLE}" -B -m unittest discover
            -s "${CMAKE_CURRENT_SOURCE_DIR}/tests" -p "test_yaris_vehicle_model.py" -v)
  set_tests_properties(yaris_vehicle_artifacts PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE
    ENVIRONMENT "YARIS_VEHICLE_ASSET_DIR=${CRASH_YARIS_VEHICLE_ASSETS}")
endif()
