cmake_minimum_required(VERSION 3.22)
project(RoboDynaRetainedTwoIntervalReplay LANGUAGES CXX)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Explicit TL root for existing bounded host utility")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_utils/BoundedArena.h")
  message(FATAL_ERROR "Retained archive replay needs explicit TL utility source root")
endif()
# Reuse the owning host record gates, including exact-cap prefix regression.
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../../../output/physical_run"
                 physical-run-qualification)
include("Replay.cmake")
