cmake_minimum_required(VERSION 3.22)
project(RoboDynaRuntimeSource LANGUAGES CXX CUDA)
enable_testing()
find_package(Chrono REQUIRED CONFIG)
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Qualified coherent TL source")
set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
include("${CMAKE_CURRENT_LIST_DIR}/../../VehiclePhysicalStartup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../vehicle_dynamics/VehiclePhysicalDynamics.cmake")
get_filename_component(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
add_executable(envelope_runtime_values "${CMAKE_CURRENT_LIST_DIR}/../../tests/PackingTest.cpp"
  "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/rigid_assembly_owner/Fixture.cpp")
target_link_libraries(envelope_runtime_values PRIVATE robo_dyna_vehicle_runtime_values tl_tied_cin_attachment GTest::gtest_main)
target_compile_options(envelope_runtime_values PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME envelope_runtime_values COMMAND envelope_runtime_values)
add_executable(envelope_runtime_actual ActualTest.cpp
  "${APP_ROOT}/case/vehicle_startup/physical_model/tests/supports/Source.cpp")
target_link_libraries(envelope_runtime_actual PRIVATE robo_dyna_vehicle_physical_dynamics GTest::gtest_main)
target_compile_options(envelope_runtime_actual PRIVATE -fno-fast-math -ffp-contract=off)
# Full source/owner tests run separately by exact filter after inspecting the
# complete forecast. CTest never starts an unrequested full source/GPU owner.
