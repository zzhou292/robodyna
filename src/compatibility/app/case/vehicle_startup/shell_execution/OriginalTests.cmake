foreach(name CANONICAL SCOPE DECLARATIONS GLASS_RESOLUTION TYPE13_DECLARATION)
  set(ROBO_DYNA_VEHICLE_${name} "" CACHE PATH "Explicit original vehicle ${name} fixture")
  if(NOT EXISTS "${ROBO_DYNA_VEHICLE_${name}}")
    message(FATAL_ERROR "Shell execution original checks require ${name}")
  endif()
endforeach()
set(ROBO_DYNA_VEHICLE_GLASS_SHA256 "" CACHE STRING "Authenticated original glass sidecar SHA256")
string(LENGTH "${ROBO_DYNA_VEHICLE_GLASS_SHA256}" sha_length)
if(NOT sha_length EQUAL 64)
  message(FATAL_ERROR "Shell execution checks require glass SHA256")
endif()
add_executable(robo_dyna_vehicle_shell_execution_original_check tests/OriginalCatalogTest.cpp
  tests/OriginalLayersTest.cpp tests/OriginalRetryTest.cpp tests/OriginalGlobalLaw1Test.cpp)
target_link_libraries(robo_dyna_vehicle_shell_execution_original_check PRIVATE
  robo_dyna_vehicle_shell_execution GTest::gtest_main)
option(ROBO_DYNA_VEHICLE_CONTACT_GEOMETRY_ORIGINAL "Verify complete source-mapped contact surface" OFF)
if(ROBO_DYNA_VEHICLE_CONTACT_GEOMETRY_ORIGINAL)
  include("${CMAKE_CURRENT_LIST_DIR}/../../shell_collection/ShellCollectionContactGeometry.cmake")
  target_sources(robo_dyna_vehicle_shell_execution_original_check PRIVATE tests/OriginalContactGeometryTest.cpp)
  target_link_libraries(robo_dyna_vehicle_shell_execution_original_check PRIVATE robo_dyna_shell_collection_contact_geometry)
endif()
target_compile_options(robo_dyna_vehicle_shell_execution_original_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_shell_execution_original COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_shell_execution_original_check>" "VehicleShellExecutionOriginal.*")
set_tests_properties(vehicle_shell_execution_original PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "ROBO_VEHICLE_DECLARATIONS=${ROBO_DYNA_VEHICLE_DECLARATIONS};ROBO_VEHICLE_GLASS_RESOLUTION=${ROBO_DYNA_VEHICLE_GLASS_RESOLUTION};ROBO_VEHICLE_GLASS_SHA256=${ROBO_DYNA_VEHICLE_GLASS_SHA256};ROBO_DYNA_TYPE13_DECLARATION=${ROBO_DYNA_VEHICLE_TYPE13_DECLARATION}")
