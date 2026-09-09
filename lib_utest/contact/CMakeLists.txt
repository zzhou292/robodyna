cmake_minimum_required(VERSION 3.18)
project(TLSurfaceContactTests LANGUAGES CXX)

# A focused, offline test harness. Bazel remains the repository-wide build.
# Use installed GTest/Eigen and compile the same source files, without building
# DEME, cuDSS, the viewer, or a complete second solver backend.
option(TL_CONTACT_ENABLE_CUDA "Build and require the CUDA contact tests" ON)
option(TL_CONTACT_ENABLE_FE_BASELINE "Build the existing two tiny ANCF3243 mass regressions" OFF)
option(TL_CONTACT_ENABLE_Q4_RECTANGULAR_QUALIFICATION "Build the separate dyadic rectangular contact candidate checks" OFF)
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

add_executable(utest_q4_surface_mapping "${TL_ROOT}/lib_utest/utest_q4_surface_mapping.cc")
target_compile_options(utest_q4_surface_mapping PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_q4_surface_mapping)

add_executable(utest_q4_contact_integration "${TL_ROOT}/lib_utest/utest_q4_contact_integration.cc")
target_compile_options(utest_q4_contact_integration PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_q4_contact_integration)

if(TL_CONTACT_ENABLE_Q4_RECTANGULAR_QUALIFICATION)
  add_executable(utest_q4_rectangular_integration "${TL_ROOT}/lib_utest/utest_q4_rectangular_integration.cc")
  target_compile_options(utest_q4_rectangular_integration PRIVATE -fno-fast-math -ffp-contract=off)
  tl_contact_test(utest_q4_rectangular_integration)
endif()

include("${TL_ROOT}/lib_src/collision/PlanarWallGeometry.cmake")
add_executable(utest_planar_wall_geometry "${TL_ROOT}/lib_utest/utest_planar_wall_geometry.cc")
target_link_libraries(utest_planar_wall_geometry PRIVATE tl_planar_wall_geometry)
target_compile_options(utest_planar_wall_geometry PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_planar_wall_geometry)

include("${TL_ROOT}/lib_src/collision/PreparedPlanarWallQuery.cmake")
add_executable(utest_prepared_planar_wall_query "${TL_ROOT}/lib_utest/utest_prepared_planar_wall_query.cc")
target_link_libraries(utest_prepared_planar_wall_query PRIVATE tl_prepared_planar_wall_query)
target_compile_options(utest_prepared_planar_wall_query PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_prepared_planar_wall_query)

add_executable(utest_planar_wall_box "${TL_ROOT}/lib_utest/utest_planar_wall_box.cc")
target_link_libraries(utest_planar_wall_box PRIVATE tl_planar_wall_geometry)
target_compile_options(utest_planar_wall_box PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_planar_wall_box)

include("${TL_ROOT}/lib_src/collision/SurfaceMaterialMeasure.cmake")
add_executable(utest_surface_material_measure "${TL_ROOT}/lib_utest/utest_surface_material_measure.cc")
target_link_libraries(utest_surface_material_measure PRIVATE tl_surface_material_measure)
target_compile_options(utest_surface_material_measure PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_surface_material_measure)

add_executable(utest_t3_contact_integration "${TL_ROOT}/lib_utest/utest_t3_contact_integration.cc")
target_link_libraries(utest_t3_contact_integration PRIVATE tl_surface_material_measure)
target_compile_options(utest_t3_contact_integration PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_t3_contact_integration)

include("${TL_ROOT}/lib_src/collision/Q4ParametricContact.cmake")
include("${TL_ROOT}/lib_src/collision/NodalWallContact.cmake")
add_executable(utest_nodal_wall_contact "${TL_ROOT}/lib_utest/utest_nodal_wall_contact.cc")
target_link_libraries(utest_nodal_wall_contact PRIVATE tl_nodal_wall_contact)
target_compile_options(utest_nodal_wall_contact PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_nodal_wall_contact)

add_executable(utest_q4_parametric_contact "${TL_ROOT}/lib_utest/utest_q4_parametric_contact.cc")
target_link_libraries(utest_q4_parametric_contact PRIVATE tl_q4_parametric_contact tl_q4_prescribed_planar_contact)
target_compile_options(utest_q4_parametric_contact PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_q4_parametric_contact)

add_executable(utest_prescribed_surface_contact "${TL_ROOT}/lib_utest/utest_prescribed_surface_contact.cc")
target_link_libraries(utest_prescribed_surface_contact PRIVATE tl_q4_parametric_contact)
target_compile_options(utest_prescribed_surface_contact PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_prescribed_surface_contact)

include("${TL_ROOT}/lib_src/collision/Q4PlanarGeometry.cmake")
add_executable(utest_q4_planar_geometry "${TL_ROOT}/lib_utest/utest_q4_planar_geometry.cc")
target_link_libraries(utest_q4_planar_geometry PRIVATE tl_q4_planar_geometry)
target_compile_options(utest_q4_planar_geometry PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_q4_planar_geometry)

add_executable(utest_q4_planar_sweep_geometry "${TL_ROOT}/lib_utest/utest_q4_planar_sweep_geometry.cc")
target_link_libraries(utest_q4_planar_sweep_geometry PRIVATE tl_q4_planar_geometry)
target_compile_options(utest_q4_planar_sweep_geometry PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_q4_planar_sweep_geometry)

include("${TL_ROOT}/lib_src/collision/Q4PrescribedPlanarContact.cmake")
foreach(check utest_q4_unrestricted_mass utest_q4_integral_measure utest_q4_prescribed_planar_contact)
  add_executable(${check} "${TL_ROOT}/lib_utest/${check}.cc")
  target_link_libraries(${check} PRIVATE tl_q4_prescribed_planar_contact)
  target_compile_options(${check} PRIVATE -fno-fast-math -ffp-contract=off)
  tl_contact_test(${check})
endforeach()

add_executable(utest_q4_planar_stiffness "${TL_ROOT}/lib_utest/utest_q4_planar_stiffness.cc")
target_link_libraries(utest_q4_planar_stiffness PRIVATE tl_q4_planar_geometry Eigen3::Eigen)
target_compile_options(utest_q4_planar_stiffness PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_q4_planar_stiffness)
set_tests_properties(utest_q4_planar_stiffness PROPERTIES PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")

add_executable(utest_step_stability "${TL_ROOT}/lib_utest/utest_step_stability.cc")
target_link_libraries(utest_step_stability PRIVATE Eigen3::Eigen)
target_compile_options(utest_step_stability PRIVATE -fno-fast-math -ffp-contract=off)
tl_contact_test(utest_step_stability)
set_tests_properties(utest_step_stability PROPERTIES PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")

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
  add_executable(utest_parametric_contact_cuda "${TL_ROOT}/lib_utest/utest_parametric_contact_cuda.cu")
  target_link_libraries(utest_parametric_contact_cuda PRIVATE tl_q4_parametric_contact CUDA::cudart)
  target_compile_options(utest_parametric_contact_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  tl_contact_test(utest_parametric_contact_cuda)
  set_tests_properties(utest_parametric_contact_cuda PROPERTIES PROCESSORS 1)
  add_executable(utest_q4_surface_mapping_cuda "${TL_ROOT}/lib_utest/utest_q4_surface_mapping_cuda.cu")
  target_link_libraries(utest_q4_surface_mapping_cuda PRIVATE CUDA::cudart)
  target_compile_options(utest_q4_surface_mapping_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_q4_surface_mapping_cuda)
  set_tests_properties(utest_q4_surface_mapping_cuda PROPERTIES PROCESSORS 1)
  add_executable(utest_q4_contact_integration_cuda "${TL_ROOT}/lib_utest/utest_q4_contact_integration_cuda.cu")
  target_link_libraries(utest_q4_contact_integration_cuda PRIVATE CUDA::cudart)
  target_compile_options(utest_q4_contact_integration_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_q4_contact_integration_cuda)
  set_tests_properties(utest_q4_contact_integration_cuda PROPERTIES PROCESSORS 1)
  if(TL_CONTACT_ENABLE_Q4_RECTANGULAR_QUALIFICATION)
    add_executable(utest_q4_rectangular_integration_cuda "${TL_ROOT}/lib_utest/utest_q4_rectangular_integration_cuda.cu")
    target_link_libraries(utest_q4_rectangular_integration_cuda PRIVATE CUDA::cudart)
    target_compile_options(utest_q4_rectangular_integration_cuda PRIVATE
      "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
    tl_contact_test(utest_q4_rectangular_integration_cuda)
    set_tests_properties(utest_q4_rectangular_integration_cuda PROPERTIES PROCESSORS 1)
  endif()
  add_executable(utest_cuda_runtime "${TL_ROOT}/lib_utest/utest_cuda_runtime.cu")
  target_link_libraries(utest_cuda_runtime PRIVATE CUDA::cudart)
  tl_contact_test(utest_cuda_runtime)

  add_executable(utest_surface_contact_cuda "${TL_ROOT}/lib_utest/utest_surface_contact_cuda.cu")
  target_link_libraries(utest_surface_contact_cuda PRIVATE CUDA::cudart)
  tl_contact_test(utest_surface_contact_cuda)

  add_executable(utest_step_stability_cuda "${TL_ROOT}/lib_utest/utest_step_stability_cuda.cu")
  target_link_libraries(utest_step_stability_cuda PRIVATE CUDA::cudart)
  target_compile_options(utest_step_stability_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_step_stability_cuda)
  set_tests_properties(utest_step_stability_cuda PROPERTIES PROCESSORS 1)

  add_subdirectory("${TL_ROOT}/lib_src/solvers" "tl-explicit-nodal-state")
  add_executable(utest_nodal_history_admission_cuda "${TL_ROOT}/lib_utest/utest_nodal_history_admission_cuda.cu")
  target_link_libraries(utest_nodal_history_admission_cuda PRIVATE tl_explicit_nodal_state)
  target_compile_options(utest_nodal_history_admission_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_nodal_history_admission_cuda)
  set_tests_properties(utest_nodal_history_admission_cuda PROPERTIES PROCESSORS 1)
  add_executable(utest_nodal_temporal_cuda "${TL_ROOT}/lib_utest/utest_nodal_temporal_cuda.cu")
  target_link_libraries(utest_nodal_temporal_cuda PRIVATE tl_explicit_nodal_state)
  target_compile_options(utest_nodal_temporal_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_nodal_temporal_cuda)
  set_tests_properties(utest_nodal_temporal_cuda PROPERTIES PROCESSORS 1)
  add_executable(utest_nodal_step_cuda "${TL_ROOT}/lib_utest/utest_nodal_step_cuda.cu")
  target_link_libraries(utest_nodal_step_cuda PRIVATE tl_explicit_nodal_state)
  target_compile_options(utest_nodal_step_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_nodal_step_cuda)
  set_tests_properties(utest_nodal_step_cuda PROPERTIES PROCESSORS 1)

  add_executable(utest_nodal_rotation_cuda "${TL_ROOT}/lib_utest/utest_nodal_rotation_cuda.cu")
  target_link_libraries(utest_nodal_rotation_cuda PRIVATE tl_explicit_nodal_state)
  target_compile_options(utest_nodal_rotation_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_nodal_rotation_cuda)
  set_tests_properties(utest_nodal_rotation_cuda PROPERTIES PROCESSORS 1)

  add_subdirectory("${TL_ROOT}/lib_src/collision" "tl-planar-mesh-contact")
  include("${TL_ROOT}/lib_src/collision/Q4PlanarContact.cmake")
  add_executable(utest_q4_planar_contact_cuda "${TL_ROOT}/lib_utest/utest_q4_planar_contact_cuda.cu")
  target_link_libraries(utest_q4_planar_contact_cuda PRIVATE tl_q4_planar_contact)
  target_compile_options(utest_q4_planar_contact_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_q4_planar_contact_cuda)
  set_tests_properties(utest_q4_planar_contact_cuda PROPERTIES PROCESSORS 1)
  add_executable(utest_planar_mesh_contact_cuda "${TL_ROOT}/lib_utest/utest_planar_mesh_contact_cuda.cu")
  target_link_libraries(utest_planar_mesh_contact_cuda PRIVATE tl_planar_mesh_contact)
  target_compile_options(utest_planar_mesh_contact_cuda PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  tl_contact_test(utest_planar_mesh_contact_cuda)
  set_tests_properties(utest_planar_mesh_contact_cuda PROPERTIES PROCESSORS 1)

  add_library(tl_contact_broadphase STATIC
    "${TL_ROOT}/lib_src/collision/HydroelasticBroadphase.cu"
    "${TL_ROOT}/lib_utils/mesh_manager.cc")
  target_include_directories(tl_contact_broadphase PUBLIC "${TL_ROOT}")
  target_link_libraries(tl_contact_broadphase PUBLIC tl_contact_cpu_utils CUDA::cudart)
  add_executable(utest_surface_broadphase "${TL_ROOT}/lib_utest/utest_surface_broadphase.cu")
  target_link_libraries(utest_surface_broadphase PRIVATE tl_contact_broadphase)
  tl_contact_test(utest_surface_broadphase)

  if(TL_CONTACT_ENABLE_FE_BASELINE)
    # Reuse the existing FE implementation and reference tests unchanged.
    # No direct solver or CUDA math-library dependency is used by this element.
    add_library(tl_contact_ancf3243 STATIC
      "${TL_ROOT}/lib_src/elements/ANCF3243Data.cu"
      "${TL_ROOT}/lib_utils/mesh_utils.cc"
      "${TL_ROOT}/lib_utils/csv_utils.cc")
    target_include_directories(tl_contact_ancf3243 PUBLIC "${TL_ROOT}")
    target_link_libraries(tl_contact_ancf3243 PUBLIC tl_contact_cpu_utils CUDA::cudart)
    target_compile_options(tl_contact_ancf3243 PRIVATE
      "$<$<COMPILE_LANGUAGE:CUDA>:-O3;--use_fast_math;--extra-device-vectorization>")
    add_executable(utest_3243 "${TL_ROOT}/lib_utest/utest_3243.cc")
    target_link_libraries(utest_3243 PRIVATE tl_contact_ancf3243)
    tl_contact_test(utest_3243)
    set_tests_properties(utest_3243 PROPERTIES WORKING_DIRECTORY "${TL_ROOT}")
  endif()
endif()
