cmake_minimum_required(VERSION 3.22)
project(RoboDynaVehicleSelfContactValues LANGUAGES CXX)
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
enable_testing()
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "TL source tree with active-use APIs")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactActiveUseBinding.cmake")
  message(FATAL_ERROR "Vehicle self-contact values require the composed TL active-use tree")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/VehicleSelfContactValues.cmake")
add_executable(robo_dyna_vehicle_self_contact_values_check
  tests/SelectedSourceTest.cpp)
target_link_libraries(robo_dyna_vehicle_self_contact_values_check PRIVATE
  robo_dyna_vehicle_self_contact_values GTest::gtest_main)
target_compile_options(robo_dyna_vehicle_self_contact_values_check PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_self_contact_values
  COMMAND robo_dyna_vehicle_self_contact_values_check)
set_tests_properties(vehicle_self_contact_values PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 90)
