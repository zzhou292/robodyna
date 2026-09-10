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
if(ROBO_DYNA_SOURCE_SHELL_COLLECTION)
  include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/ShellBatchBinding.cmake")
  add_library(robo_dyna_source_shell_collection STATIC SourceShellCollection.cpp)
  target_link_libraries(robo_dyna_source_shell_collection PUBLIC
    robo_dyna_source_contact_fixture tl_shell_batch_binding)
  target_compile_options(robo_dyna_source_shell_collection PRIVATE -fno-fast-math -ffp-contract=off)
  add_executable(robo_dyna_source_shell_collection_check source_shell_collection_check.cpp)
  target_link_libraries(robo_dyna_source_shell_collection_check PRIVATE
    robo_dyna_source_shell_collection GTest::gtest)
  target_include_directories(robo_dyna_source_shell_collection_check PRIVATE
    "${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/native/t3")
  target_compile_options(robo_dyna_source_shell_collection_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_shell_collection COMMAND robo_dyna_source_shell_collection_check
    "${ROBO_DYNA_SOURCE_PART_READINESS}")
  set_tests_properties(source_shell_collection PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)
endif()
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
option(ROBO_DYNA_SOURCE_NODAL_WALL "Qualify source nodal finite-wall contact and CUDA cost" OFF)
if(ROBO_DYNA_SOURCE_CONTACT_CUDA OR ROBO_DYNA_SOURCE_NODAL_WALL)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 120)
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()
if(ROBO_DYNA_SOURCE_CONTACT_CUDA)
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

if(ROBO_DYNA_SOURCE_NODAL_WALL)
  include("${CMAKE_CURRENT_LIST_DIR}/SourceNodalWall.cmake")
endif()

# Optional native startup oracle; it consumes the existing authenticated source
# fixture and has no CUDA or original MAT024 mechanics admission.
if(ROBO_DYNA_SOURCE_T3_STARTUP OR ROBO_DYNA_SOURCE_T3_RATES OR ROBO_DYNA_SOURCE_T3_FORCE)
  enable_language(Fortran)
  # Seven standalone native tests are qualified in their own small build. This
  # app opt-in registers only the two source-six tests against the same library.
  set(T3_R1_BUILD_TESTS OFF)
  set(T3_R2_BUILD ${ROBO_DYNA_SOURCE_T3_RATES})
  if(ROBO_DYNA_SOURCE_T3_FORCE)
    set(T3_R2_BUILD ON)
  endif()
  set(T3_R2_BUILD_TESTS OFF)
  set(T3_R3_BUILD ${ROBO_DYNA_SOURCE_T3_FORCE})
  set(T3_R3_BUILD_TESTS OFF)
  add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/native/t3"
    "${CMAKE_CURRENT_BINARY_DIR}/native-t3-r1" EXCLUDE_FROM_ALL)
endif()

if(ROBO_DYNA_SOURCE_T3_FORCE)
  add_executable(robo_dyna_source_part_t3_force_check source_part_t3_force_check.cpp)
  target_link_libraries(robo_dyna_source_part_t3_force_check PRIVATE
    robo_dyna_source_contact_fixture t3_r3_native GTest::gtest)
  target_compile_options(robo_dyna_source_part_t3_force_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_part_t3_force COMMAND robo_dyna_source_part_t3_force_check
    "${ROBO_DYNA_SOURCE_PART_READINESS}")
  set_tests_properties(source_part_t3_force PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
if(ROBO_DYNA_SOURCE_T3_STARTUP)
  add_executable(robo_dyna_source_part_t3_startup_check source_part_t3_startup_check.cpp)
  target_link_libraries(robo_dyna_source_part_t3_startup_check PRIVATE
    robo_dyna_source_contact_fixture t3_r1_native GTest::gtest)
  target_compile_options(robo_dyna_source_part_t3_startup_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_part_t3_startup COMMAND robo_dyna_source_part_t3_startup_check
    "${ROBO_DYNA_SOURCE_PART_READINESS}")
  set_tests_properties(source_part_t3_startup PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()

if(ROBO_DYNA_SOURCE_T3_RATES)
  add_executable(robo_dyna_source_part_t3_rates_check source_part_t3_rates_check.cpp)
  target_link_libraries(robo_dyna_source_part_t3_rates_check PRIVATE
    robo_dyna_source_contact_fixture t3_r2_native GTest::gtest)
  target_compile_options(robo_dyna_source_part_t3_rates_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_part_t3_rates COMMAND robo_dyna_source_part_t3_rates_check
    "${ROBO_DYNA_SOURCE_PART_READINESS}")
  set_tests_properties(source_part_t3_rates PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()

if(ROBO_DYNA_SOURCE_QEPH_GEOMETRY)
  enable_language(Fortran)
  set(QEPH_Q1_BUILD_TESTS OFF)
  add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements/qeph"
    "${CMAKE_CURRENT_BINARY_DIR}/tl-qeph-source" EXCLUDE_FROM_ALL)
  add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/native/qeph"
    "${CMAKE_CURRENT_BINARY_DIR}/native-qeph-source" EXCLUDE_FROM_ALL)
  add_executable(robo_dyna_source_part_qeph_geometry_check source_part_qeph_geometry_check.cpp)
  target_link_libraries(robo_dyna_source_part_qeph_geometry_check PRIVATE
    robo_dyna_source_contact_fixture tl_qeph qeph_q1_native GTest::gtest)
  target_compile_options(robo_dyna_source_part_qeph_geometry_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_part_qeph_geometry COMMAND robo_dyna_source_part_qeph_geometry_check
    "${ROBO_DYNA_SOURCE_PART_READINESS}")
  set_tests_properties(source_part_qeph_geometry PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()

option(ROBO_DYNA_SOURCE_PART_WALL_GEOMETRY "Check mass-independent source contact geometry and explicit placed original wall" OFF)
if(ROBO_DYNA_SOURCE_PART_WALL_GEOMETRY)
  include("${CMAKE_CURRENT_LIST_DIR}/../../case/source_part_wall/SourcePartWallGeometryChecks.cmake")
endif()
