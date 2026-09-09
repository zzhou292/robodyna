find_package(GTest REQUIRED)
set(CRASH_TL_FEA_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../Total-Lagrangian-FEA"
  CACHE PATH "Owning TL-FEA source tree")
set(ROBO_DYNA_SOURCE_PART_READINESS "" CACHE FILEPATH "Authenticated E2a source-part readiness artifact")
set(CRASH_CANONICAL_WALL "" CACHE FILEPATH "Authenticated canonical Yaris wall manifest")
foreach(input ROBO_DYNA_SOURCE_PART_READINESS CRASH_CANONICAL_WALL)
  if(NOT EXISTS "${${input}}")
    message(FATAL_ERROR "${input} must name its prepared artifact; no fetch or skipped gate")
  endif()
endforeach()
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/collision/SurfaceMaterialMeasure.cmake")
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/collision/Q4PlanarGeometry.cmake")
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/collision/Q4ParametricContact.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../case/WallTessellationLibrary.cmake")
add_library(robo_dyna_source_contact_fixture STATIC SourcePartContactFixture.cpp)
target_include_directories(robo_dyna_source_contact_fixture PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}" "${CRASH_TL_FEA_SOURCE_DIR}/lib_src")
target_link_libraries(robo_dyna_source_contact_fixture PUBLIC robo_dyna_artifact_io)
target_compile_options(robo_dyna_source_contact_fixture PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_contact_check source_part_contact_check.cpp)
target_link_libraries(robo_dyna_source_part_contact_check PRIVATE robo_dyna_source_contact_fixture
  tl_surface_material_measure robo_dyna_wall_tessellation robo_dyna_canonical_wall_artifacts GTest::gtest)
target_compile_options(robo_dyna_source_part_contact_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME source_part_contact_geometry COMMAND robo_dyna_source_part_contact_check
  "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
set_tests_properties(source_part_contact_geometry PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")

add_library(robo_dyna_source_contact_force_fixture STATIC SourceContactForceFixture.cpp)
target_link_libraries(robo_dyna_source_contact_force_fixture PUBLIC robo_dyna_source_contact_fixture
  tl_q4_parametric_contact)
target_compile_options(robo_dyna_source_contact_force_fixture PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_force_check source_part_force_check.cpp)
target_link_libraries(robo_dyna_source_part_force_check PRIVATE robo_dyna_source_contact_force_fixture
  robo_dyna_wall_tessellation robo_dyna_canonical_wall_artifacts GTest::gtest)
target_compile_options(robo_dyna_source_part_force_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME source_part_contact_force COMMAND robo_dyna_source_part_force_check
  "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
set_tests_properties(source_part_contact_force PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")

option(ROBO_DYNA_SOURCE_CONTACT_CUDA "Require actual CUDA source-parent contact qualification" OFF)
if(ROBO_DYNA_SOURCE_CONTACT_CUDA)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 120)
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_executable(robo_dyna_source_part_contact_cuda_check
    SourceContactCudaFixture.cu source_part_contact_cuda_check.cpp)
  set_target_properties(robo_dyna_source_part_contact_cuda_check PROPERTIES
    CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_link_libraries(robo_dyna_source_part_contact_cuda_check PRIVATE
    robo_dyna_source_contact_force_fixture robo_dyna_wall_tessellation
    robo_dyna_canonical_wall_artifacts CUDA::cudart GTest::gtest)
  target_compile_options(robo_dyna_source_part_contact_cuda_check PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME source_part_contact_cuda COMMAND robo_dyna_source_part_contact_cuda_check
    "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
  set_tests_properties(source_part_contact_cuda PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
