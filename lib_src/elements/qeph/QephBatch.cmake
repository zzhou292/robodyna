# Optional resident QEPH participant; composing build enables CUDA and supplies
# tl_qeph and tl_explicit_nodal_state. No native/reference/test dependency.
include("${CMAKE_CURRENT_LIST_DIR}/../ShellBatchBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ShellBatchPlasticity.cmake")
add_library(tl_qeph_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/QephBatch.cu"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchOperations.cu"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchKernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchModel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchIdentity.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchPublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchPlasticity.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchSections.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchFailure.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/QephBatchFailureReadback.cpp")
target_link_libraries(tl_qeph_batch PUBLIC tl_shell_batch_plasticity)
target_link_libraries(tl_qeph_batch PUBLIC tl_qeph tl_explicit_nodal_state tl_shell_batch_binding)
set_target_properties(tl_qeph_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_qeph_batch PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")

include("${CMAKE_CURRENT_LIST_DIR}/../../assembly/NodalMassBinding.cmake")
target_link_libraries(tl_qeph_batch PUBLIC tl_nodal_mass_binding)
