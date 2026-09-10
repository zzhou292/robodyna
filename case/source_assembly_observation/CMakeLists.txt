cmake_minimum_required(VERSION 3.22)
project(RoboDynaSourceAssemblyObservation LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyObservation.cmake")
set(ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY "" CACHE FILEPATH "Frozen six-part Yaris source inventory")
if(NOT EXISTS "${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY}")
  message(FATAL_ERROR "Observation checks require the authenticated source inventory")
endif()
add_executable(robo_dyna_source_assembly_observation_check
  tests/SourceAssemblyKineticTest.cpp tests/SourceAssemblyKickTest.cpp
  tests/SourceAssemblyForceStageTest.cpp tests/SourceAssemblyForceStageFailureTest.cpp
  tests/SourceAssemblyQephSpinTest.cpp)
target_link_libraries(robo_dyna_source_assembly_observation_check PRIVATE
  robo_dyna_source_assembly_observation GTest::gtest_main)
target_compile_options(robo_dyna_source_assembly_observation_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME source_assembly_observation COMMAND robo_dyna_source_assembly_observation_check)
set_tests_properties(source_assembly_observation PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60
  ENVIRONMENT "ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY}")

include("${CMAKE_CURRENT_LIST_DIR}/../../output/source_assembly/QephSpinFields.cmake")
add_executable(robo_dyna_source_assembly_spin_fields_check
  "${CMAKE_CURRENT_LIST_DIR}/../../output/source_assembly/tests/QephSpinFieldsTest.cpp")
target_link_libraries(robo_dyna_source_assembly_spin_fields_check PRIVATE robo_dyna_source_assembly_spin_fields GTest::gtest_main)
target_compile_options(robo_dyna_source_assembly_spin_fields_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME source_assembly_spin_fields COMMAND robo_dyna_source_assembly_spin_fields_check)
set_tests_properties(source_assembly_spin_fields PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60
  ENVIRONMENT "ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY}")
