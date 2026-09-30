include_guard(GLOBAL)
if(NOT TARGET tl_explicit_nodal_state)
  message(FATAL_ERROR "Beam18 Batch requires the existing common nodal owner")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../../ShellPhysicalOwner.cmake")
add_library(tl_beam18_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Batch.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Candidate.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Assembly.cu"
  "${CMAKE_CURRENT_LIST_DIR}/AssembleOperation.cu"
  "${CMAKE_CURRENT_LIST_DIR}/EvaluateOperation.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Readback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Publication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PublicationScope.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/OutputRanges.cpp")
target_link_libraries(tl_beam18_batch PUBLIC tl_beam18_batch_values
  tl_shell_physical_owner tl_explicit_nodal_state)
target_compile_features(tl_beam18_batch PUBLIC cxx_std_17)
set_target_properties(tl_beam18_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_beam18_batch PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
