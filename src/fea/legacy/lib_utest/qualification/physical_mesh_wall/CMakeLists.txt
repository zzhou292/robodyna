cmake_minimum_required(VERSION 3.22)
project(TLPhysicalMeshWall LANGUAGES CXX C)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
option(PHYSICAL_MESH_WALL_CUDA "Actual all-family mapped contact owner gate" OFF)
include("${TL_ROOT}/lib_src/assembly/ShellNodeMap.cmake")
include("${TL_ROOT}/lib_src/collision/NodalWallContact.cmake")
include("${TL_ROOT}/lib_src/collision/PreparedPlanarWallQuery.cmake")
add_executable(physical_mesh_wall_values ValueTest.cpp HostPreparationTest.cpp
  "${TL_ROOT}/lib_src/collision/nodal_wall_mapped/Layout.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactModel.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactArena.cpp"
  "${TL_ROOT}/lib_src/collision/NodalWallContactIncidence.cpp")
target_include_directories(physical_mesh_wall_values PRIVATE "${TL_ROOT}" "${CUDAToolkit_INCLUDE_DIRS}")
target_compile_features(physical_mesh_wall_values PRIVATE cxx_std_17)
target_compile_options(physical_mesh_wall_values PRIVATE -fno-fast-math -ffp-contract=off
  -ffunction-sections -fdata-sections)
# The existing Model TU also owns legacy CUDA initialization; this host-only
# gate discards that unused entry without linking or executing a device owner.
target_link_options(physical_mesh_wall_values PRIVATE -Wl,--gc-sections)
target_link_libraries(physical_mesh_wall_values PRIVATE GTest::gtest_main
  tl_shell_node_map tl_nodal_wall_contact tl_prepared_planar_wall_query)
add_test(NAME physical_mesh_wall_values COMMAND physical_mesh_wall_values)
if(PHYSICAL_MESH_WALL_CUDA)
  enable_language(CUDA)
  set(PHYSICAL_PUBLICATION_CUDA ON CACHE BOOL "Reuse complete owner fixture" FORCE)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../physical_publication" physical-publication)
  include("${TL_ROOT}/lib_src/collision/NodalWallMappedContact.cmake")
  add_executable(physical_mesh_wall_cuda OwnerTest.cu ParticipationTest.cu
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerStartup.cu"
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerAttempt.cu"
    "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerSnapshot.cu")
  target_link_libraries(physical_mesh_wall_cuda PRIVATE physical_publication_fixture tl_nodal_wall_mapped CUDA::cudart)
  set_target_properties(physical_mesh_wall_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(physical_mesh_wall_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME physical_mesh_wall_cuda COMMAND physical_mesh_wall_cuda)
  set_tests_properties(physical_mesh_wall_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 240)
endif()
