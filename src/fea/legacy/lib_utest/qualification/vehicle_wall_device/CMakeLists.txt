cmake_minimum_required(VERSION 3.22)
project(TLVehicleWallDevice LANGUAGES CXX)
enable_testing()
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
include("${TL_ROOT}/lib_src/collision/NodalWallContact.cmake")
include("${TL_ROOT}/lib_src/collision/PreparedPlanarWallQuery.cmake")
include("${TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
add_executable(vehicle_wall_device_host_check
  VehicleWallLayoutTest.cpp VehicleWallStartupTest.cpp VehicleWallSmallSourceTest.cpp
  VehicleWallFixture.cpp VehicleWallChecks.cpp
  "${TL_ROOT}/lib_utest/qualification/surface_contact/NodalWallCapacityFixture.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactArena.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactModel.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactIncidence.cpp")
target_include_directories(vehicle_wall_device_host_check PRIVATE "${TL_ROOT}" "${CUDAToolkit_INCLUDE_DIRS}")
target_link_libraries(vehicle_wall_device_host_check PRIVATE tl_nodal_wall_contact tl_prepared_planar_wall_query tl_shell_batch_binding CUDA::cudart GTest::gtest_main)
# Drop unused CUDA-owner entry points from the reused model/fixture translation
# units. This host gate invokes only immutable PrepareModel and layout methods.
target_compile_options(vehicle_wall_device_host_check PRIVATE -fno-fast-math -ffp-contract=off -ffunction-sections -fdata-sections)
target_link_options(vehicle_wall_device_host_check PRIVATE "LINKER:--gc-sections")
add_test(NAME vehicle_wall_device_host_check COMMAND vehicle_wall_device_host_check)
set_tests_properties(vehicle_wall_device_host_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
option(TL_VEHICLE_WALL_DEVICE_CUDA "Build complete-count contact owner qualification" OFF)
if(TL_VEHICLE_WALL_DEVICE_CUDA)
  enable_language(CUDA)
  add_subdirectory("${TL_ROOT}/lib_src/solvers" "tl-nodal-owner")
  include("${TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
  include("${TL_ROOT}/lib_src/collision/NodalWallContactDevice.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/VehicleWallCuda.cmake")
endif()
