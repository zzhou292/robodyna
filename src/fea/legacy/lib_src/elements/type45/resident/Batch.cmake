include_guard(GLOBAL)
if(NOT TARGET tl_explicit_nodal_state)
  message(FATAL_ERROR "TYPE45 Batch requires the existing common nodal owner")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/Values.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../ShellPhysicalOwner.cmake")
add_library(tl_type45_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Batch.cu" "${CMAKE_CURRENT_LIST_DIR}/Candidate.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Assembly.cu" "${CMAKE_CURRENT_LIST_DIR}/Contexts.cu"
  "${CMAKE_CURRENT_LIST_DIR}/AssembleOperation.cu" "${CMAKE_CURRENT_LIST_DIR}/EvaluateOperation.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Readback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Publication.cpp" "${CMAKE_CURRENT_LIST_DIR}/PublicationScope.cpp")
target_link_libraries(tl_type45_batch PUBLIC tl_type45_batch_values tl_shell_physical_owner tl_explicit_nodal_state)
target_compile_features(tl_type45_batch PUBLIC cxx_std_17)
set_target_properties(tl_type45_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_type45_batch PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
