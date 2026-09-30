enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)
add_subdirectory("${tl_root}/lib_src/solvers" nodal-owner)
include("${tl_root}/lib_src/elements/qeph/QephBatch.cmake")
include("${tl_root}/lib_src/elements/t3/T3Batch.cmake")
include("${tl_root}/lib_src/elements/ShellBatchPublication.cmake")
add_executable(qbat_binding_closed_participants_test ClosedParticipantsTest.cu)
target_link_libraries(qbat_binding_closed_participants_test PRIVATE tl_shell_batch_publication
  tl_nodal_mass_binding GTest::gtest_main CUDA::cudart)
set_target_properties(qbat_binding_closed_participants_test PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(qbat_binding_closed_participants_test PRIVATE
  --fmad=false --prec-div=true --prec-sqrt=true --ftz=false
  -Xcompiler=-fno-fast-math,-ffp-contract=off)
add_test(NAME qbat_binding_closed_participants COMMAND qbat_binding_closed_participants_test)
