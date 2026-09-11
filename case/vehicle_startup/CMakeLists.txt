cmake_minimum_required(VERSION 3.18)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RoboDynaVehicleShellReferences LANGUAGES CXX)
  find_package(Chrono REQUIRED CONFIG)
  enable_testing()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/VehicleShellReferences.cmake")
option(ROBO_DYNA_VEHICLE_REFERENCE_TESTS "Build native reference host checks" ON)
option(ROBO_DYNA_VEHICLE_REFERENCE_RESOLVED_TESTS "Build resolved full-source reference checks" OFF)
option(ROBO_DYNA_VEHICLE_REFERENCE_SOURCE_TESTS "Build actual full-source reference checks" OFF)
option(ROBO_DYNA_VEHICLE_REFERENCE_GLASS_TESTS "Build original glass reference checks" OFF)
if(ROBO_DYNA_VEHICLE_REFERENCE_TESTS)
  find_package(GTest REQUIRED)
  add_executable(robo_dyna_vehicle_reference_value_check tests/ReferenceValueTest.cpp)
  target_link_libraries(robo_dyna_vehicle_reference_value_check PRIVATE robo_dyna_vehicle_native_reference GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_reference_value_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_reference_values COMMAND robo_dyna_vehicle_reference_value_check)
endif()

if(ROBO_DYNA_VEHICLE_REFERENCE_GLASS_TESTS)
  find_package(GTest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  foreach(name CANONICAL SCOPE DECLARATIONS RESOLUTION GLASS_RESOLUTION)
    set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
    if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
      message(FATAL_ERROR "Glass reference checks require explicit ${name} fixture")
    endif()
  endforeach()
  set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated original glass sidecar SHA256")
  string(LENGTH "${ROBO_DYNA_VEHICLE_GLASS_SHA256}" glass_sha_length)
  if(NOT glass_sha_length EQUAL 64)
    message(FATAL_ERROR "Glass reference checks require its explicit SHA256")
  endif()
  add_executable(robo_dyna_vehicle_glass_reference_check tests/GlassReferenceSourceTest.cpp)
  target_link_libraries(robo_dyna_vehicle_glass_reference_check PRIVATE robo_dyna_vehicle_shell_references GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_glass_reference_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_glass_reference COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_vehicle_glass_reference_check>" "VehicleGlassReference.*")
  set_tests_properties(vehicle_glass_reference PROPERTIES PROCESSORS 1 RUN_SERIAL TRUE TIMEOUT 180
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_RESOLUTION=${ROBO_DYNA_VEHICLE_RESOLUTION};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256}")
endif()
if(ROBO_DYNA_VEHICLE_REFERENCE_SOURCE_TESTS OR ROBO_DYNA_VEHICLE_REFERENCE_RESOLVED_TESTS)
  find_package(GTest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  include("${CMAKE_CURRENT_LIST_DIR}/../../modelio/source_assembly/SourceAssembly.cmake")
  include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
  add_executable(robo_dyna_vehicle_reference_source_check tests/ReferenceSourceTest.cpp tests/ReferenceProjectionTest.cpp)
  target_link_libraries(robo_dyna_vehicle_reference_source_check PRIVATE robo_dyna_vehicle_shell_references
    robo_dyna_source_assembly_shell_input tl_shell_batch_binding GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_reference_source_check PRIVATE -fno-fast-math -ffp-contract=off)
  foreach(name CANONICAL SCOPE DECLARATIONS ELASTIC MIXED)
    set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
    if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
      message(FATAL_ERROR "Vehicle native checks require explicit ${name} fixture")
    endif()
  endforeach()
  add_test(NAME vehicle_reference_source COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_vehicle_reference_source_check>" "VehicleReferenceSource.*")
  set_tests_properties(vehicle_reference_source PROPERTIES PROCESSORS 1 RUN_SERIAL TRUE TIMEOUT 180
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_ELASTIC=${ROBO_DYNA_VEHICLE_ELASTIC};ROBO_VEHICLE_MIXED=${ROBO_DYNA_VEHICLE_MIXED}")
endif()

if(ROBO_DYNA_VEHICLE_REFERENCE_RESOLVED_TESTS)
  set(ROBO_DYNA_VEHICLE_RESOLUTION "" CACHE FILEPATH "Explicit immutable vehicle section resolution")
  if(NOT EXISTS "${ROBO_DYNA_VEHICLE_RESOLUTION}")
    message(FATAL_ERROR "Resolved vehicle reference checks require explicit RESOLUTION fixture")
  endif()
  add_executable(robo_dyna_vehicle_reference_resolved_check tests/ResolvedReferenceSourceTest.cpp
    tests/ResolvedReferenceParityTest.cpp)
  target_link_libraries(robo_dyna_vehicle_reference_resolved_check PRIVATE robo_dyna_vehicle_shell_references
    robo_dyna_source_assembly_shell_input tl_shell_batch_binding GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_reference_resolved_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_reference_resolved COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
    "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
    "$<TARGET_FILE:robo_dyna_vehicle_reference_resolved_check>" "VehicleResolvedReference.*")
  set_tests_properties(vehicle_reference_resolved PROPERTIES PROCESSORS 1 RUN_SERIAL TRUE TIMEOUT 180
    ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_ELASTIC=${ROBO_DYNA_VEHICLE_ELASTIC};ROBO_VEHICLE_MIXED=${ROBO_DYNA_VEHICLE_MIXED};ROBO_VEHICLE_RESOLUTION=${ROBO_DYNA_VEHICLE_RESOLUTION}")
endif()
