# Optional standalone resident T3 participant. Composing build supplies CUDA,
# tl_t3 and tl_explicit_nodal_state; no native/reference/test dependency.
include("${CMAKE_CURRENT_LIST_DIR}/../ShellBatchBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ShellBatchPlasticity.cmake")
add_library(tl_t3_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/T3Batch.cu"
  "${CMAKE_CURRENT_LIST_DIR}/T3BatchOperations.cu"
  "${CMAKE_CURRENT_LIST_DIR}/T3BatchKernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/T3BatchModel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/T3BatchIdentity.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/T3BatchPublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/T3BatchPlasticity.cpp")
target_link_libraries(tl_t3_batch PUBLIC tl_shell_batch_plasticity)
target_link_libraries(tl_t3_batch PUBLIC tl_t3 tl_explicit_nodal_state tl_shell_batch_binding)
set_target_properties(tl_t3_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_t3_batch PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
