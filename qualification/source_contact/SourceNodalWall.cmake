# Optional source-wall qualification and phase diagnosis; no production runner.
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/collision/NodalWallContact.cmake")
add_library(robo_dyna_source_nodal_wall_fixture STATIC SourceNodalWallFixture.cpp)
target_link_libraries(robo_dyna_source_nodal_wall_fixture PUBLIC
  robo_dyna_source_contact_force_fixture tl_nodal_wall_contact)
target_compile_options(robo_dyna_source_nodal_wall_fixture PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_nodal_wall_check
  SourceNodalWallCuda.cu SourceContactCudaFixture.cu source_part_nodal_wall_check.cpp
  source_part_nodal_wall_cuda_check.cpp source_part_nodal_wall_cost_check.cpp)
add_executable(robo_dyna_source_nodal_wall_profile
  SourceNodalWallCuda.cu source_nodal_wall_profile.cpp)
foreach(nodal_target IN ITEMS robo_dyna_source_part_nodal_wall_check robo_dyna_source_nodal_wall_profile)
  set_target_properties(${nodal_target} PROPERTIES
    CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_link_libraries(${nodal_target} PRIVATE
    robo_dyna_source_nodal_wall_fixture robo_dyna_wall_tessellation
    robo_dyna_canonical_wall_artifacts CUDA::cudart GTest::gtest)
  target_compile_options(${nodal_target} PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr;--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
endforeach()
add_test(NAME source_part_nodal_wall COMMAND robo_dyna_source_part_nodal_wall_check
  "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
set_tests_properties(source_part_nodal_wall PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")

add_test(NAME source_nodal_wall_profile COMMAND robo_dyna_source_nodal_wall_profile
  "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
set_tests_properties(source_nodal_wall_profile PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
