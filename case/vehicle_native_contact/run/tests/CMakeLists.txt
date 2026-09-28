cmake_minimum_required(VERSION 3.22)
project(RoboDynaNativeArchiveRequest LANGUAGES CXX)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Qualified TL source for existing bounded storage utilities")
get_filename_component(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include("${APP_ROOT}/output/physical_run/PhysicalRun.cmake")
include("${APP_ROOT}/case/vehicle_native_contact/actual/controls/Controls.cmake")
add_executable(native_archive_request_check
  ArchiveRequestTest.cpp OriginalSourceChunkTest.cpp "${CMAKE_CURRENT_LIST_DIR}/../ArchiveRequest.cpp"
  "${APP_ROOT}/output/physical_run/tests/NativeGroupRecordsTest.cpp"
  "${APP_ROOT}/output/physical_run/tests/ReplayBudgetTest.cpp"
  "${APP_ROOT}/output/physical_run/tests/ReplayPreflightTest.cpp"
  "${APP_ROOT}/output/full_shell/static_bundle/tests/SourceChunkTest.cpp"
  "${APP_ROOT}/case/vehicle_native_contact/actual/controls/tests/PreviewControlsTest.cpp"
  "${APP_ROOT}/case/vehicle_native_contact/actual/controls/tests/StageTimingFlagTest.cpp")
target_link_libraries(native_archive_request_check PRIVATE robo_dyna_physical_run_records
  robo_dyna_native_preview_controls GTest::gtest_main)
target_compile_options(native_archive_request_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME native_archive_request COMMAND native_archive_request_check
  --gtest_filter=-NativeArchiveOriginalSource.*)
set_tests_properties(native_archive_request PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
set(ROBO_DYNA_SOURCE_CANONICAL "" CACHE PATH "Original Yaris canonical fixture")
set(ROBO_DYNA_SOURCE_SCOPE "" CACHE FILEPATH "Original Yaris scope fixture")
if(ROBO_DYNA_SOURCE_CANONICAL AND ROBO_DYNA_SOURCE_SCOPE)
  add_test(NAME native_archive_original_source COMMAND "${Python3_EXECUTABLE}" -B
    "${APP_ROOT}/output/full_shell/static_bundle/tests/actual_source_fixture.py"
    "${ROBO_DYNA_SOURCE_CANONICAL}" "${ROBO_DYNA_SOURCE_SCOPE}"
    "$<TARGET_FILE:native_archive_request_check>" "NativeArchiveOriginalSource.*")
  set_tests_properties(native_archive_original_source PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
endif()
