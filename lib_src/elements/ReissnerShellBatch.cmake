# Optional production CUDA batch. The composing build enables CUDA and provides
# tl_explicit_nodal_state; no tests, case data or Chrono dependency enter TL.
add_library(tl_reissner_shell_batch STATIC "${CMAKE_CURRENT_LIST_DIR}/ReissnerShellBatch.cu")
target_link_libraries(tl_reissner_shell_batch PUBLIC tl_reissner_shell tl_explicit_nodal_state)
set_target_properties(tl_reissner_shell_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_reissner_shell_batch PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
