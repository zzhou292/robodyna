cmake_minimum_required(VERSION 3.22)
project(RoboDynaNativeVehicleCase LANGUAGES CXX CUDA)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Explicit qualified General TYPE25 source")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
include("${CMAKE_CURRENT_LIST_DIR}/../VehicleContactStartup.cmake")
get_filename_component(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
add_executable(native_vehicle_contact_actual ActualTest.cpp Report.cpp
  "${APP_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
target_link_libraries(native_vehicle_contact_actual PRIVATE robo_dyna_vehicle_native_contact GTest::gtest_main)
target_compile_options(native_vehicle_contact_actual PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(native_vehicle_case_fields "${CMAKE_CURRENT_LIST_DIR}/../tests/FieldPackingTest.cpp")
target_link_libraries(native_vehicle_case_fields PRIVATE robo_dyna_native_vehicle_field_packing GTest::gtest_main)
target_compile_options(native_vehicle_case_fields PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME native_vehicle_case_fields COMMAND native_vehicle_case_fields)
set_tests_properties(native_vehicle_case_fields PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
# The complete-source tests are invoked individually under the owning18GiB
# guard, with a fresh output destination. CTest never starts a full source/GPU
# census or physical owner implicitly.
