# Explicit original-source opt-in. This never changes a case factory's policy.
add_executable(robo_dyna_vehicle_solid_extended_check tests/ExtendedSourceTest.cpp)
target_link_libraries(robo_dyna_vehicle_solid_extended_check PRIVATE
  robo_dyna_vehicle_solid_source GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_solid_extended_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_solid_extended_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
  "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
  "$<TARGET_FILE:robo_dyna_vehicle_solid_extended_check>" "VehicleSolidExtendedSource.*")

if(ROBO_DYNA_VEHICLE_SOLID_EXTENDED_NATIVE)
  enable_language(Fortran)
  foreach(family solid24 solid6z)
    # Existing complete native libraries remain in their owning TL directory.
    if(NOT TARGET ${family}_reference_native)
      add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/${family}_reference/native"
        "${CMAKE_CURRENT_BINARY_DIR}/extended_${family}_native")
    endif()
    add_executable(robo_dyna_vehicle_${family}_extended_native_check tests/ExtendedNativeTest.cpp)
    target_link_libraries(robo_dyna_vehicle_${family}_extended_native_check PRIVATE
      robo_dyna_vehicle_solid_source ${family}_reference_native GTest::gtest_main)
    target_compile_options(robo_dyna_vehicle_${family}_extended_native_check PRIVATE
      -fno-fast-math -ffp-contract=off)
    if(family STREQUAL "solid24")
      target_compile_definitions(robo_dyna_vehicle_${family}_extended_native_check PRIVATE
        ROBO_DYNA_EXTENDED_SOLID24)
    endif()
    add_test(NAME vehicle_${family}_extended_native COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell/static_bundle/tests/actual_source_fixture.py"
      "${ROBO_DYNA_VEHICLE_CANONICAL}" "${ROBO_DYNA_VEHICLE_SCOPE}"
      "$<TARGET_FILE:robo_dyna_vehicle_${family}_extended_native_check>" "VehicleSolidExtendedNative.*")
  endforeach()
endif()
