include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/QbatBatchValues.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ShellPhysicalOwner.cmake")
add_library(tl_qbat_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/QbatBatch.cu"
  "${CMAKE_CURRENT_LIST_DIR}/QbatBatchOperations.cu"
  "${CMAKE_CURRENT_LIST_DIR}/QbatBatchReadback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/QbatBatchKernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/QbatBatchPublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Initialize.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Assemble.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Kernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Measurement.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Publication.cpp")
target_link_libraries(tl_qbat_batch PUBLIC tl_qbat_batch_values tl_explicit_nodal_state tl_shell_physical_owner)
set_target_properties(tl_qbat_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_qbat_batch PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
