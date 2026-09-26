cmake_minimum_required(VERSION 3.22)
project(RoboDynaVehicleContactNodalSeed LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
include("${CMAKE_CURRENT_LIST_DIR}/../ContactNodalSeed.cmake")
get_filename_component(ROBO_SEED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
add_executable(vehicle_contact_seed_values tests/SlotMapTest.cpp tests/DigestTest.cpp)
target_link_libraries(vehicle_contact_seed_values PRIVATE robo_dyna_vehicle_contact_nodal_seed GTest::gtest_main)
target_compile_options(vehicle_contact_seed_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_contact_seed_values COMMAND vehicle_contact_seed_values)
set_tests_properties(vehicle_contact_seed_values PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
add_executable(vehicle_contact_seed_consumer tests/Consumer.cpp)
target_link_libraries(vehicle_contact_seed_consumer PRIVATE robo_dyna_vehicle_contact_nodal_seed)
add_test(NAME vehicle_contact_seed_consumer COMMAND vehicle_contact_seed_consumer)
# The extracted set reader must preserve every existing selection regression.
add_subdirectory("${ROBO_SEED_ROOT}/modelio/self_contact" original-selection-regression)

option(ROBO_DYNA_VEHICLE_CONTACT_SEED_ACTUAL_TESTS "Build explicit full V5 pre-correction source gate" OFF)
if(ROBO_DYNA_VEHICLE_CONTACT_SEED_ACTUAL_TESTS)
  add_executable(vehicle_contact_seed_actual tests/ActualTest.cpp
    "${ROBO_SEED_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
  target_link_libraries(vehicle_contact_seed_actual PRIVATE robo_dyna_vehicle_contact_nodal_seed GTest::gtest_main)
  target_compile_options(vehicle_contact_seed_actual PRIVATE -fno-fast-math -ffp-contract=off)
  # The complete source fixture is intentionally not an automatic CTest. The
  # lead owns separate guarded forecast and create-only preparation commands.
endif()

option(ROBO_DYNA_VEHICLE_CONTACT_SEED_NATIVE_TESTS "Independent pinned LAW42 contact-slot witness" ON)
if(ROBO_DYNA_VEHICLE_CONTACT_SEED_NATIVE_TESTS)
  enable_language(Fortran)
  include("${CMAKE_CURRENT_LIST_DIR}/tests/native/Native.cmake")
  add_executable(vehicle_contact_seed_material_native tests/MaterialSlotsTest.cpp)
  target_link_libraries(vehicle_contact_seed_material_native PRIVATE robo_dyna_vehicle_contact_nodal_seed
    robo_dyna_law42_contact_slots_native GTest::gtest_main)
  target_compile_options(vehicle_contact_seed_material_native PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME vehicle_contact_seed_material_native COMMAND vehicle_contact_seed_material_native)
  set_tests_properties(vehicle_contact_seed_material_native law42_contact_slots_source
    PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
endif()
