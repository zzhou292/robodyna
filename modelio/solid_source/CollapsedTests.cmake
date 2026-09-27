# Complete source qualification only; no case-factory or owner enable.
enable_language(Fortran)
if(NOT TARGET solid24_reference_native)
  add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/solid24_reference/native"
    "${CMAKE_CURRENT_BINARY_DIR}/collapsed_solid24_native")
endif()
add_executable(robo_dyna_vehicle_collapsed_solid_source_check tests/CollapsedSourceTest.cpp)
target_link_libraries(robo_dyna_vehicle_collapsed_solid_source_check PRIVATE
  robo_dyna_vehicle_solid_source solid24_reference_native GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_collapsed_solid_source_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_collapsed_solid_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_collapsed_solid_source_check>" "VehicleCollapsedSolidSource.*")
