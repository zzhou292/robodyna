cmake_minimum_required(VERSION 3.22)
project(RoboDynaTiedPostKinChk LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/../TiedSearchPostKinChkValues.cmake")
find_package(GTest REQUIRED)
add_executable(robo_dyna_tied_post_kinchk_values_check tests/ValueTest.cpp
  "${CMAKE_CURRENT_LIST_DIR}/../../../modelio/tied_shell/tests/TinyFixture.cpp")
target_link_libraries(robo_dyna_tied_post_kinchk_values_check PRIVATE
  robo_dyna_tied_post_kinchk_values GTest::gtest_main)
add_test(NAME tied_post_kinchk_values COMMAND robo_dyna_tied_post_kinchk_values_check)
option(ROBO_DYNA_TIED_POST_KINCHK_ACTUAL "Build original observed post-KINCHK/native gate" OFF)
if(ROBO_DYNA_TIED_POST_KINCHK_ACTUAL)
  enable_language(CUDA)
  enable_language(Fortran)
  include("${CMAKE_CURRENT_LIST_DIR}/../TiedSearchPostKinChk.cmake")
  include("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/tied_post_kinchk/Native.cmake")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(ROBO_DYNA_TIED_CANONICAL "" CACHE PATH "Original canonical directory")
  set(ROBO_DYNA_TIED_SCOPE "" CACHE FILEPATH "Original scope10 report")
  set(ROBO_DYNA_VEHICLE_DECLARATIONS "" CACHE FILEPATH "Immutable V1 vehicle declarations")
  foreach(input ROBO_DYNA_TIED_CANONICAL ROBO_DYNA_TIED_SCOPE ROBO_DYNA_VEHICLE_DECLARATIONS)
    if(NOT EXISTS "${${input}}")
      message(FATAL_ERROR "Original post-KINCHK fixture missing: ${input}")
    endif()
  endforeach()
  add_executable(robo_dyna_tied_post_kinchk_actual_check tests/ActualSourceTest.cpp
    "${CMAKE_CURRENT_LIST_DIR}/../tied_classification/tests/ActualClassification.cpp")
  target_link_libraries(robo_dyna_tied_post_kinchk_actual_check PRIVATE
    robo_dyna_tied_search_post_kinchk tied_post_kinchk_oracle GTest::gtest_main)
  add_test(NAME tied_post_kinchk_actual COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../tied_classification/tests/actual_fixture.py"
    "${ROBO_DYNA_TIED_CANONICAL}" "${ROBO_DYNA_TIED_SCOPE}" "${ROBO_DYNA_VEHICLE_DECLARATIONS}"
    "$<TARGET_FILE:robo_dyna_tied_post_kinchk_actual_check>" "TiedPostKinChkActual.*")
  set_tests_properties(tied_post_kinchk_actual PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
endif()
