cmake_minimum_required(VERSION 3.18)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RoboDynaTiedSearchGeometry LANGUAGES CXX)
  find_package(Chrono REQUIRED CONFIG)
  enable_testing()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../TiedShellSearchGeometry.cmake")
find_package(GTest REQUIRED)
add_executable(robo_dyna_tied_search_geometry_check tests/ValueTest.cpp ../../tied_shell/tests/TinyFixture.cpp)
target_link_libraries(robo_dyna_tied_search_geometry_check PRIVATE robo_dyna_tied_search_geometry GTest::gtest_main)
add_test(NAME tied_search_geometry_values COMMAND robo_dyna_tied_search_geometry_check --gtest_filter=TiedSearchGeometryValues.*)
option(ROBO_DYNA_TIED_SEARCH_GEOMETRY_NATIVE "Build complete independent INCOQ3 oracle" OFF)
set(TL_SOURCE_ROOT "${ROBO_DYNA_TL_ROOT}" CACHE PATH "Explicit TL source verifier owner")
if(ROBO_DYNA_TIED_SEARCH_GEOMETRY_NATIVE)
  enable_language(Fortran)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/verify_sources.py"
    "${TL_SOURCE_ROOT}" RESULT_VARIABLE native_status)
  if(NOT native_status EQUAL 0)
    message(FATAL_ERROR "Pinned INCOQ3 source verification failed")
  endif()
  add_library(robo_dyna_tied_search_geometry_native STATIC native/Context.F90 native/Interfaces.F90
    native/Packet.F90 native/ConsumedThickness.F native/original/common_source/modules/constant_mod.F
    native/original/starter/source/interfaces/inter3d1/incoq3.F)
  set_target_properties(robo_dyna_tied_search_geometry_native PROPERTIES
    Fortran_MODULE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/native_modules")
  target_include_directories(robo_dyna_tied_search_geometry_native PRIVATE native native/include
    native/original/engine/share/r8 "${CMAKE_CURRENT_BINARY_DIR}/native_modules")
  target_compile_options(robo_dyna_tied_search_geometry_native PRIVATE -cpp -ffixed-line-length-none
    -ffree-line-length-none -fno-fast-math -ffp-contract=off -fcheck=all -finit-real=snan)
  target_sources(robo_dyna_tied_search_geometry_check PRIVATE tests/NativeTest.cpp tests/NativeOracle.cpp tests/ActualSourceTest.cpp)
  target_link_libraries(robo_dyna_tied_search_geometry_check PRIVATE robo_dyna_tied_search_geometry_native)
  add_test(NAME tied_search_geometry_native COMMAND robo_dyna_tied_search_geometry_check --gtest_filter=TiedSearchGeometryNative.*)
  set(ROBO_DYNA_TIED_CANONICAL "" CACHE PATH "Explicit original canonical directory")
  set(ROBO_DYNA_TIED_SCOPE "" CACHE FILEPATH "Explicit original scope10")
  if(ROBO_DYNA_TIED_CANONICAL AND ROBO_DYNA_TIED_SCOPE)
    add_test(NAME tied_search_geometry_actual COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
      "${ROBO_DYNA_TIED_CANONICAL}" "${ROBO_DYNA_TIED_SCOPE}"
      "$<TARGET_FILE:robo_dyna_tied_search_geometry_check>" "TiedSearchGeometryActual.*")
    set_tests_properties(tied_search_geometry_actual PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
  endif()
endif()
