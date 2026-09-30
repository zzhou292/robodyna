# Focused affected legacy consumers of the common scratch issuer/publication.
# Reuse their owning fixtures and test bodies; do not repeat long vehicle gates.
include("${TL_ROOT}/lib_src/collision/NodalWallMappedContact.cmake")
include("${TL_ROOT}/lib_src/collision/SelfContactTransaction.cmake")
set(_native_legacy_owner OwnerStartup.cu OwnerAttempt.cu OwnerSnapshot.cu)
add_executable(native_contact_wall_legacy_cuda ${_native_legacy_owner}
  "${CMAKE_CURRENT_LIST_DIR}/../physical_mesh_wall/OwnerTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_mesh_wall/ParticipationTest.cu")
target_link_libraries(native_contact_wall_legacy_cuda PRIVATE physical_publication_fixture tl_nodal_wall_mapped CUDA::cudart)
add_executable(native_contact_self_legacy_cuda ${_native_legacy_owner}
  "${CMAKE_CURRENT_LIST_DIR}/../self_contact_transaction/CudaTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../self_contact_transaction/FacetFilterCudaProbe.cpp")
target_link_libraries(native_contact_self_legacy_cuda PRIVATE physical_publication_fixture tl_self_contact_transaction CUDA::cudart GTest::gtest_main)
target_link_options(native_contact_self_legacy_cuda PRIVATE -Wl,--wrap=cudaMalloc -Wl,--wrap=cudaMemcpyAsync)
foreach(target native_contact_wall_legacy_cuda native_contact_self_legacy_cuda)
  set_target_properties(${target} PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(${target} PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
endforeach()
add_test(NAME native_contact_wall_legacy_cuda COMMAND native_contact_wall_legacy_cuda)
add_test(NAME native_contact_self_legacy_cuda COMMAND native_contact_self_legacy_cuda
  "--gtest_filter=SelfContactTransactionCuda.ExactForecastCapMinusOneAndRosterEntryAreStable:SelfContactTransactionCuda.SingleParentZeroPairZeroEventStillParticipatesAndCommits:SelfContactTransactionCuda.AcceptedInteriorEeForceCandidateRetryAndRollbackKeepForceSti:SelfContactTransactionCuda.ActualT3RemovalFiltersCandidateAndLongInactiveRetryCommits")
set_tests_properties(native_contact_wall_legacy_cuda native_contact_self_legacy_cuda PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 240)
