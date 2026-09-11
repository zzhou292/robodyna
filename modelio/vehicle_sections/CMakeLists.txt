cmake_minimum_required(VERSION 3.18)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(RoboDynaVehicleSections LANGUAGES CXX)
  find_package(Chrono REQUIRED CONFIG)
  enable_testing()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/VehicleSectionResolution.cmake")
option(ROBO_DYNA_VEHICLE_SECTION_TESTS "Build vehicle section declaration host checks" ON)
option(ROBO_DYNA_VEHICLE_SECTION_ACTUAL_TESTS "Enable explicit complete original vehicle checks" OFF)
option(ROBO_DYNA_VEHICLE_GLASS_ACTUAL_TESTS "Enable original glass resolution checks" OFF)
if(ROBO_DYNA_VEHICLE_SECTION_TESTS)
  find_package(GTest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(field_fixture "${CMAKE_CURRENT_BINARY_DIR}/section-fields.json")
  add_custom_command(OUTPUT "${field_fixture}"
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/tests/field_fixture.py" "${field_fixture}"
    DEPENDS tests/field_fixture.py ../../modelio/constant_failure_declarations.py
      ../../modelio/vehicle_section_resolution.py)
  add_executable(robo_dyna_vehicle_section_fields_check tests/ResolutionFieldsTest.cpp "${field_fixture}")
  target_link_libraries(robo_dyna_vehicle_section_fields_check PRIVATE robo_dyna_vehicle_sections GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_section_fields_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_section_fields COMMAND robo_dyna_vehicle_section_fields_check)
  set_tests_properties(vehicle_section_fields PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "ROBO_VEHICLE_SECTION_FIELDS=${field_fixture}")
  set(glass_fixture "${CMAKE_CURRENT_BINARY_DIR}/glass-fields.json")
  add_custom_command(OUTPUT "${glass_fixture}"
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/tests/glass_fixture.py" "${glass_fixture}"
    DEPENDS tests/glass_fixture.py ../../tests/test_glass_declarations.py
      ../../modelio/glass_declarations.py ../../modelio/vehicle_section_resolution.py)
  add_executable(robo_dyna_vehicle_glass_fields_check tests/GlassFieldsTest.cpp "${glass_fixture}")
  target_link_libraries(robo_dyna_vehicle_glass_fields_check PRIVATE robo_dyna_vehicle_sections GTest::gtest_main)
  target_compile_options(robo_dyna_vehicle_glass_fields_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_glass_fields COMMAND robo_dyna_vehicle_glass_fields_check)
  set_tests_properties(vehicle_glass_fields PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "ROBO_GLASS_FIELDS=${glass_fixture}")
  if(ROBO_DYNA_VEHICLE_SECTION_ACTUAL_TESTS)
    foreach(name CANONICAL SCOPE DECLARATIONS RESOLUTION)
      set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
      if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
        message(FATAL_ERROR "Vehicle section checks require explicit ${name} fixture")
      endif()
    endforeach()
    add_executable(robo_dyna_vehicle_section_resolution_check tests/ResolutionActualTest.cpp
      tests/ResolutionRejectionTest.cpp)
    target_link_libraries(robo_dyna_vehicle_section_resolution_check PRIVATE robo_dyna_vehicle_sections GTest::gtest_main)
    target_compile_options(robo_dyna_vehicle_section_resolution_check PRIVATE -fno-fast-math -ffp-contract=off)
    add_test(NAME vehicle_section_resolution COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_vehicle_section_resolution_check>" "VehicleSectionResolution.*")
    set_tests_properties(vehicle_section_resolution PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1
      ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_RESOLUTION=${ROBO_DYNA_VEHICLE_RESOLUTION}")
  endif()
  if(ROBO_DYNA_VEHICLE_GLASS_ACTUAL_TESTS)
    foreach(name CANONICAL SCOPE DECLARATIONS RESOLUTION GLASS_RESOLUTION)
      set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
      if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
        message(FATAL_ERROR "Glass resolution checks require explicit ${name} fixture")
      endif()
    endforeach()
    set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated original glass sidecar SHA256")
    string(LENGTH "${ROBO_DYNA_VEHICLE_GLASS_SHA256}" glass_sha_length)
    if(NOT glass_sha_length EQUAL 64)
      message(FATAL_ERROR "Glass resolution checks require its explicit SHA256")
    endif()
    add_executable(robo_dyna_vehicle_glass_source_check tests/GlassSourceTest.cpp)
    target_link_libraries(robo_dyna_vehicle_glass_source_check PRIVATE robo_dyna_vehicle_sections GTest::gtest_main)
    target_compile_options(robo_dyna_vehicle_glass_source_check PRIVATE -fno-fast-math -ffp-contract=off)
    add_test(NAME vehicle_glass_source COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_vehicle_glass_source_check>" "VehicleGlassResolution.*")
    set_tests_properties(vehicle_glass_source PROPERTIES TIMEOUT 180 RUN_SERIAL TRUE PROCESSORS 1
      ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_RESOLUTION=${ROBO_DYNA_VEHICLE_RESOLUTION};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256}")
  endif()
endif()
