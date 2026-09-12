cmake_minimum_required(VERSION 3.22)
project(RoboDynaNativeStageReports LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Chrono REQUIRED CONFIG)
find_package(CUDAToolkit REQUIRED)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Qualified TL source checkout")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/elements/beam18/resident/Batch.h")
  message(FATAL_ERROR "ROBO_DYNA_TL_ROOT must contain the qualified beam and solid report types")
endif()
include("../../../vehicle_run/Values.cmake")
add_executable(robo_dyna_native_stage_reports_check
  "../../NativeStageError.cpp" ReportsTest.cpp LoopFailureTest.cpp)
target_include_directories(robo_dyna_native_stage_reports_check PRIVATE
  "${ROBO_DYNA_TL_ROOT}" "${ROBO_DYNA_TL_ROOT}/lib_src"
  "${CUDAToolkit_INCLUDE_DIRS}")
target_link_libraries(robo_dyna_native_stage_reports_check PRIVATE
  robo_dyna_vehicle_run_values GTest::gtest_main Eigen3::Eigen Chrono::Chrono_core)
target_compile_options(robo_dyna_native_stage_reports_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_native_stage_reports COMMAND robo_dyna_native_stage_reports_check)
