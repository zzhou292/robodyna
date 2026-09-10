cmake_minimum_required(VERSION 3.18)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RoboDynaFullShellFrameGeometry LANGUAGES CXX)
  find_package(Chrono REQUIRED CONFIG)
  enable_testing()
endif()
if(NOT TARGET robo_dyna_full_shell_source_mapping)
  set(ROBO_DYNA_FULL_SHELL_RECORD_TESTS OFF CACHE BOOL "Build record tests in their owning gate")
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell" "${CMAKE_CURRENT_BINARY_DIR}/records")
endif()
if(NOT TARGET robo_dyna_replay_geometry_values)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../replay" "${CMAKE_CURRENT_BINARY_DIR}/replay-core")
endif()
add_library(robo_dyna_full_shell_frame_geometry STATIC FullShellFrameGeometry.cpp
  FrameGeometryInitialize.cpp FrameGeometryChecks.cpp FrameGeometryFields.cpp)
target_link_libraries(robo_dyna_full_shell_frame_geometry PUBLIC
  robo_dyna_full_shell_source_mapping robo_dyna_replay_geometry_values)
target_compile_features(robo_dyna_full_shell_frame_geometry PUBLIC cxx_std_17)
target_compile_options(robo_dyna_full_shell_frame_geometry PRIVATE -fno-fast-math -ffp-contract=off)
option(ROBO_DYNA_FULL_SHELL_FRAME_GEOMETRY_TESTS "Build presentation-only source/binary frame tests" ON)
if(ROBO_DYNA_FULL_SHELL_FRAME_GEOMETRY_TESTS)
  find_package(GTest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(robo_dyna_full_shell_frame_geometry_check tests/ScalarApplicabilityTest.cpp
    tests/FrameGeometryValueTest.cpp tests/FrameGeometryActualTest.cpp)
  target_link_libraries(robo_dyna_full_shell_frame_geometry_check PRIVATE
    robo_dyna_full_shell_frame_geometry GTest::gtest_main)
  target_compile_options(robo_dyna_full_shell_frame_geometry_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME full_shell_frame_geometry_values COMMAND robo_dyna_full_shell_frame_geometry_check
    --gtest_filter=-FullShellGeometryActual.*)
  set_tests_properties(full_shell_frame_geometry_values PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)
  set(ROBO_DYNA_FRAME_GEOMETRY_CANONICAL_FIXTURE "" CACHE PATH "Explicit authenticated original canonical source")
  set(ROBO_DYNA_FRAME_GEOMETRY_SCOPE_FIXTURE "" CACHE FILEPATH "Explicit authenticated no-tire report")
  if(ROBO_DYNA_FRAME_GEOMETRY_CANONICAL_FIXTURE OR ROBO_DYNA_FRAME_GEOMETRY_SCOPE_FIXTURE)
    if(NOT EXISTS "${ROBO_DYNA_FRAME_GEOMETRY_CANONICAL_FIXTURE}/manifest.json" OR
       NOT EXISTS "${ROBO_DYNA_FRAME_GEOMETRY_CANONICAL_FIXTURE}/source_model.zip" OR
       NOT EXISTS "${ROBO_DYNA_FRAME_GEOMETRY_SCOPE_FIXTURE}")
      message(FATAL_ERROR "Complete explicit original source fixture paths are required")
    endif()
    add_test(NAME full_shell_frame_geometry_actual COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
      "${ROBO_DYNA_FRAME_GEOMETRY_CANONICAL_FIXTURE}" "${ROBO_DYNA_FRAME_GEOMETRY_SCOPE_FIXTURE}"
      "$<TARGET_FILE:robo_dyna_full_shell_frame_geometry_check>" "FullShellGeometryActual.*")
    set_tests_properties(full_shell_frame_geometry_actual PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1)
  endif()
endif()
