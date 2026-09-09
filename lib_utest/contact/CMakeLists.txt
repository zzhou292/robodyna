cmake_minimum_required(VERSION 3.18)
project(TLSurfaceContactTests LANGUAGES CXX)

# A focused, offline test harness. Bazel remains the repository-wide build.
# Use installed GTest/Eigen and compile the same source files, without building
# DEME, cuDSS, the viewer, or a complete second solver backend.
option(TL_CONTACT_ENABLE_CUDA "Build and require the CUDA contact tests" ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
find_package(GTest REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
enable_testing()

function(tl_contact_test target)
  target_include_directories(${target} PRIVATE "${TL_ROOT}")
  target_link_libraries(${target} PRIVATE GTest::gtest_main)
  add_test(NAME ${target} COMMAND ${target})
  set_tests_properties(${target} PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
endfunction()

add_executable(utest_surface_contact "${TL_ROOT}/lib_utest/utest_surface_contact.cc")
tl_contact_test(utest_surface_contact)

add_executable(utest_surface_geometry "${TL_ROOT}/lib_utest/utest_surface_geometry.cc")
tl_contact_test(utest_surface_geometry)

add_library(tl_contact_cpu_utils STATIC "${TL_ROOT}/lib_utils/cpu_utils.cc")
target_include_directories(tl_contact_cpu_utils PUBLIC "${TL_ROOT}")
target_link_libraries(tl_contact_cpu_utils PUBLIC Eigen3::Eigen)
add_executable(utest_utils "${TL_ROOT}/lib_utest/utest_utils.cc")
target_link_libraries(utest_utils PRIVATE tl_contact_cpu_utils)
tl_contact_test(utest_utils)

if(TL_CONTACT_ENABLE_CUDA)
  # Set explicitly for reproducibility. Override at configure time for another
  # workstation. Do not compile every supported GPU architecture.
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 120)
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  add_executable(utest_cuda_runtime "${TL_ROOT}/lib_utest/utest_cuda_runtime.cu")
  target_link_libraries(utest_cuda_runtime PRIVATE CUDA::cudart)
  tl_contact_test(utest_cuda_runtime)

  add_executable(utest_surface_contact_cuda "${TL_ROOT}/lib_utest/utest_surface_contact_cuda.cu")
  target_link_libraries(utest_surface_contact_cuda PRIVATE CUDA::cudart)
  tl_contact_test(utest_surface_contact_cuda)

  add_library(tl_contact_broadphase STATIC
    "${TL_ROOT}/lib_src/collision/HydroelasticBroadphase.cu"
    "${TL_ROOT}/lib_utils/mesh_manager.cc")
  target_include_directories(tl_contact_broadphase PUBLIC "${TL_ROOT}")
  target_link_libraries(tl_contact_broadphase PUBLIC tl_contact_cpu_utils CUDA::cudart)
  add_executable(utest_surface_broadphase "${TL_ROOT}/lib_utest/utest_surface_broadphase.cu")
  target_link_libraries(utest_surface_broadphase PRIVATE tl_contact_broadphase)
  tl_contact_test(utest_surface_broadphase)
endif()
