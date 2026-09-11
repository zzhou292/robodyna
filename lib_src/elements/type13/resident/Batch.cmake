# Caller enables CUDA and provides the single tl_explicit_nodal_state target.
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/Values.cmake")
add_library(tl_type13_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Batch.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Assembly.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Candidate.cu"
  "${CMAKE_CURRENT_LIST_DIR}/AssembleOperation.cu"
  "${CMAKE_CURRENT_LIST_DIR}/EvaluateOperation.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Readback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Publication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PublicationScope.cpp")
target_link_libraries(tl_type13_batch PUBLIC tl_type13_batch_values tl_explicit_nodal_state)
target_compile_features(tl_type13_batch PUBLIC cxx_std_17)
set_target_properties(tl_type13_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_type13_batch PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
