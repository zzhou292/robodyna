cmake_minimum_required(VERSION 3.22)
project(TLRigidAssemblyOwner LANGUAGES CXX)
option(TL_RIGID_ASSEMBLY_OWNER_CUDA "Build the actual owner and native recurrence gate" OFF)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/constraints/NodalRigidAssemblyBinding.cmake")
include("${TL_ROOT}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
add_library(rigid_owner_host_storage STATIC
  "${TL_ROOT}/lib_src/solvers/NodalRigidAssemblyStorage.cpp"
  "${TL_ROOT}/lib_src/solvers/NodalRigidAssemblyChecks.cpp"
  "${TL_ROOT}/lib_src/solvers/NodalCinStartup.cpp"
  "${TL_ROOT}/lib_src/solvers/NodalCinState.cpp")
target_link_libraries(rigid_owner_host_storage PUBLIC tl_nodal_rigid_assembly_binding tl_tied_cin_attachment CUDA::cudart)
target_compile_options(rigid_owner_host_storage PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(rigid_assembly_owner_host HostTest.cpp Fixture.cpp
  ../nodal_rigid_group/VehicleRigidHostTest.cpp
  ../tied_cin_runtime/ValueTest.cpp ../tied_cin_runtime/StartupTest.cpp
  ../nodal_vehicle/LayoutTest.cpp)
target_link_libraries(rigid_assembly_owner_host PRIVATE rigid_owner_host_storage GTest::gtest_main)
target_compile_options(rigid_assembly_owner_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME rigid_assembly_owner_host COMMAND rigid_assembly_owner_host)
set_tests_properties(rigid_assembly_owner_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
if(TL_RIGID_ASSEMBLY_OWNER_CUDA)
  include("${CMAKE_CURRENT_LIST_DIR}/OwnerCuda.cmake")
endif()
