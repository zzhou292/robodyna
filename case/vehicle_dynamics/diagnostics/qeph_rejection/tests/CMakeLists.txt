cmake_minimum_required(VERSION 3.22)
project(RoboDynaQephRejectedInput LANGUAGES CXX)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "TL source with bounded QEPH rejected input diagnostics")
get_filename_component(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../.." ABSOLUTE)
include("${APP_ROOT}/output/physical_run/PhysicalRun.cmake")
include("${APP_ROOT}/case/vehicle_native_contact/actual/controls/Controls.cmake")
add_executable(qeph_rejected_input_check CodecTest.cpp ExportTest.cpp
  "${CMAKE_CURRENT_LIST_DIR}/../Codec.cpp" "${CMAKE_CURRENT_LIST_DIR}/../CaptureValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../Export.cpp"
  "${ROBO_DYNA_TL_ROOT}/lib_src/elements/qeph/rejected_candidate/Replay.cpp"
  "${APP_ROOT}/case/vehicle_native_contact/actual/controls/tests/PreviewControlsTest.cpp"
  "${APP_ROOT}/case/vehicle_native_contact/actual/controls/tests/StageTimingFlagTest.cpp")
target_include_directories(qeph_rejected_input_check PRIVATE "${APP_ROOT}" "${ROBO_DYNA_TL_ROOT}"
  "${ROBO_DYNA_TL_ROOT}/lib_src" "${CUDAToolkit_INCLUDE_DIRS}")
target_link_libraries(qeph_rejected_input_check PRIVATE robo_dyna_physical_run_records
  robo_dyna_native_preview_controls GTest::gtest_main Eigen3::Eigen)
target_compile_options(qeph_rejected_input_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qeph_rejected_input_host COMMAND qeph_rejected_input_check)
set_tests_properties(qeph_rejected_input_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
