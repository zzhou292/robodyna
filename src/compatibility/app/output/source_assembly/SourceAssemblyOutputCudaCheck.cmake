# Optional actual-source GPU contract. Reuse the existing qualification fixture;
# it is test-only and does not become an application solver dependency.
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)
include("${robo_assembly_output_root}/case/source_assembly/SourceAssemblyBindings.cmake")
include("${robo_assembly_output_root}/case/shell_collection/ShellCollectionContactGeometry.cmake")
if(NOT TARGET tl_explicit_nodal_state)
  add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_src/solvers" "tl-owner")
endif()
if(NOT TARGET tl_qeph_batch)
  include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/qeph/QephBatch.cmake")
endif()
if(NOT TARGET tl_t3_batch)
  include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/t3/T3Batch.cmake")
endif()
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchPublication.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyAcceptedOutput.cmake")
add_executable(robo_dyna_source_assembly_accepted_output_check
  "${CMAKE_CURRENT_LIST_DIR}/tests/SourceAssemblyAcceptedOutputTest.cpp"
  "${robo_assembly_output_root}/qualification/source_assembly/SourceAssemblyFlightFixture.cu")
target_link_libraries(robo_dyna_source_assembly_accepted_output_check PRIVATE
  robo_dyna_source_assembly_accepted_output robo_dyna_shell_collection_contact_geometry GTest::gtest_main CUDA::cudart)
set_target_properties(robo_dyna_source_assembly_accepted_output_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo_dyna_source_assembly_accepted_output_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME source_assembly_output_cuda COMMAND robo_dyna_source_assembly_accepted_output_check)
set_tests_properties(source_assembly_output_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180
  ENVIRONMENT "ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY}")
