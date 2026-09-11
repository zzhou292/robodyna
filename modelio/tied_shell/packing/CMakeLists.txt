cmake_minimum_required(VERSION 3.18)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RoboDynaTiedShellPacking LANGUAGES CXX)
  find_package(Chrono REQUIRED CONFIG)
  enable_testing()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../TiedShellPacking.cmake")
find_package(GTest REQUIRED)
add_executable(robo_dyna_tied_packing_check tests/ValueTest.cpp ../tests/TinyFixture.cpp)
target_link_libraries(robo_dyna_tied_packing_check PRIVATE robo_dyna_tied_shell_packing GTest::gtest_main)
add_test(NAME tied_packing_values COMMAND robo_dyna_tied_packing_check --gtest_filter=TiedPackingValues.*)
option(ROBO_DYNA_TIED_PACKING_NATIVE "Build independent pinned native packing oracle" OFF)
if(ROBO_DYNA_TIED_PACKING_NATIVE)
  enable_language(C)
  enable_language(Fortran)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(native_generated "${CMAKE_CURRENT_BINARY_DIR}/native_generated")
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/generate.py"
    "${native_generated}" RESULT_VARIABLE native_status)
  if(NOT native_status EQUAL 0)
    message(FATAL_ERROR "Pinned tied packing sources failed verification")
  endif()
  add_library(robo_dyna_tied_packing_native STATIC native/Context.F90 native/Packet.F90
    "${native_generated}/my_orders.c" "${native_generated}/inpoint.F"
    "${native_generated}/insurf.F" "${native_generated}/count3.F"
    "${native_generated}/shell_surface_buffer.F")
  target_include_directories(robo_dyna_tied_packing_native PRIVATE "${native_generated}")
  target_compile_options(robo_dyna_tied_packing_native PRIVATE
    "$<$<COMPILE_LANGUAGE:Fortran>:-ffixed-line-length-none;-ffree-line-length-none>")
  target_sources(robo_dyna_tied_packing_check PRIVATE tests/NativeOracle.cpp tests/NativeTest.cpp tests/ActualSourceTest.cpp)
  target_link_libraries(robo_dyna_tied_packing_check PRIVATE robo_dyna_tied_packing_native)
  add_test(NAME tied_packing_native COMMAND robo_dyna_tied_packing_check --gtest_filter=TiedPackingNative.*)
  set(ROBO_DYNA_TIED_CANONICAL "" CACHE PATH "Explicit original canonical directory")
  set(ROBO_DYNA_TIED_SCOPE "" CACHE FILEPATH "Explicit original scope10")
  if(ROBO_DYNA_TIED_CANONICAL AND ROBO_DYNA_TIED_SCOPE)
    add_test(NAME tied_packing_actual COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
      "${ROBO_DYNA_TIED_CANONICAL}" "${ROBO_DYNA_TIED_SCOPE}"
      "$<TARGET_FILE:robo_dyna_tied_packing_check>" "TiedPackingActual.*")
    set_tests_properties(tied_packing_actual PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
  endif()
endif()
