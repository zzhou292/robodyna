cmake_minimum_required(VERSION 3.22)
project(RoboDynaEnvelopeAcceptedOutput LANGUAGES CXX CUDA)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Qualified coherent TL source")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
get_filename_component(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(ROBO_DYNA_PHYSICAL_CAPTURE_RUNTIME ON CACHE BOOL "Real complete-owner readback" FORCE)
set(ROBO_DYNA_GLOBAL_LAW1_CAPTURE ON CACHE BOOL "Preserve real mixed-owner field regression" FORCE)
add_subdirectory("${APP_ROOT}/output/physical_frames" "capture")
set(ROBO_DYNA_PHYSICAL_RUN_LIVE_FACTORY ON CACHE BOOL "Actual accepted interval factory" FORCE)
set(ROBO_DYNA_PHYSICAL_RUN_ENVIRONMENT ON CACHE BOOL "Declared physical environment archive" FORCE)
add_subdirectory("${APP_ROOT}/output/physical_run" "records")
add_executable(envelope_output_actual ActualTest.cpp
  "${APP_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
target_link_libraries(envelope_output_actual PRIVATE robo_dyna_physical_run_environment
  robo_dyna_physical_run_live GTest::gtest_main)
target_compile_options(envelope_output_actual PRIVATE -fno-fast-math -ffp-contract=off)
# Full source/GPU/capture calls are explicit filters after the complete forecast.
# CTest includes only the small field/record and existing LAW1 owner gates.
