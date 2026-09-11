# Closed mixed-shell publication; the composing project provides both typed
# batches and the sole nodal owner. Native oracles are test-only dependencies.
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/type25/Type25Batch.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/qbat/QbatBatch.cmake")
add_library(tl_shell_batch_publication STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPublicationValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchFormulationStartup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchFormulationPreflight.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchFormulationPublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPublicationKernels.cu")
target_link_libraries(tl_shell_batch_publication PUBLIC
  tl_qeph_batch tl_t3_batch tl_type25_batch tl_qbat_batch tl_shell_batch_binding tl_explicit_nodal_state)
set_target_properties(tl_shell_batch_publication PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_shell_batch_publication PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
