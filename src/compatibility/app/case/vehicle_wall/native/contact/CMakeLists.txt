cmake_minimum_required(VERSION 3.22)
project(RoboDynaFiniteWallContactSource LANGUAGES CXX C CUDA Fortran)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(CUDAToolkit REQUIRED)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Explicit coherent qualified TL source")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
include("${CMAKE_CURRENT_LIST_DIR}/../FiniteWallContactSource.cmake")
set(TYPE25_CONTRIBUTIONS_CUDA OFF)
set(TYPE25_FRICTION_CUDA OFF)
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_nodal_seed" seed-reference)
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_friction" friction-reference)
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_fixed_main_startup" topology-reference)
# Reuse the existing complete gap oracle without adding its duplicate shell
# regression target, already brought in by the nodal seed owning project.
set(gap_root "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_gap_source")
set(gap_native "${CMAKE_CURRENT_BINARY_DIR}/gap-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${gap_root}/native/prepare.py"
  --output "${gap_native}" COMMAND_ERROR_IS_FATAL ANY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${gap_root}/native/prepare.py" "${gap_root}/native/Gap.F.in" "${gap_root}/native/source-manifest.json")
add_library(finite_wall_gap_native STATIC "${gap_root}/NativeOracle.cpp" "${gap_native}/Gap.F")
target_link_libraries(finite_wall_gap_native PUBLIC tl_radioss_type25_gap_source)
target_compile_options(finite_wall_gap_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-ffixed-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
add_executable(finite_wall_contact_values tests/ValuesTest.cpp tests/TopologyTest.cpp)
target_link_libraries(finite_wall_contact_values PRIVATE robo_dyna_finite_wall_contact_source
  type25_seed_oracle type25_coefficient_oracle finite_wall_gap_native type25_friction_oracle type25_startup_oracle GTest::gtest_main)
target_compile_options(finite_wall_contact_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME finite_wall_contact_values COMMAND finite_wall_contact_values)
add_test(NAME finite_wall_gap_native_source COMMAND "${Python3_EXECUTABLE}" -B
  "${gap_root}/native/prepare.py" --output "${gap_native}" --check)
add_executable(finite_wall_contact_consumer tests/Consumer.cpp)
target_link_libraries(finite_wall_contact_consumer PRIVATE robo_dyna_finite_wall_contact_source)
add_test(NAME finite_wall_contact_consumer COMMAND finite_wall_contact_consumer)
set_tests_properties(finite_wall_contact_values finite_wall_contact_consumer finite_wall_gap_native_source
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)

option(ROBO_DYNA_FINITE_WALL_SOURCE_ACTUAL "Build explicit full-case wall source forecast/actual gates" OFF)
if(ROBO_DYNA_FINITE_WALL_SOURCE_ACTUAL)
  get_filename_component(app "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
  add_executable(finite_wall_contact_actual tests/ActualTest.cpp
    "${app}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(finite_wall_contact_actual PRIVATE robo_dyna_finite_wall_contact_source GTest::gtest_main)
  target_compile_options(finite_wall_contact_actual PRIVATE -fno-fast-math -ffp-contract=off)
  # Exact separate guarded calls after complete coexistence forecast review.
endif()
