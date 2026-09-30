# Existing resident sources; caller provides values and the common owner.
include_guard(GLOBAL)
if(NOT TARGET tl_explicit_nodal_state)
  message(FATAL_ERROR "Solid Batch requires the existing common nodal owner target")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../../ShellPhysicalOwner.cmake")
add_library(tl_solid_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/measurement/Begin.h"
  "${CMAKE_CURRENT_LIST_DIR}/measurement/Tile.h"
  "${CMAKE_CURRENT_LIST_DIR}/measurement/Read.h"
  "${CMAKE_CURRENT_LIST_DIR}/measurement/Family.cuh"
  "${CMAKE_CURRENT_LIST_DIR}/measurement/Finalize.cuh"
  "${CMAKE_CURRENT_LIST_DIR}/Batch.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Candidate.cu"
  "${CMAKE_CURRENT_LIST_DIR}/controlled/Kernel.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResultValidation.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Assembly.cu"
  "${CMAKE_CURRENT_LIST_DIR}/AssembleOperation.cu"
  "${CMAKE_CURRENT_LIST_DIR}/EvaluateOperation.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Readback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Publication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PublicationScope.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/OutputRanges.cpp")
target_link_libraries(tl_solid_batch PUBLIC tl_solid_batch_values
  tl_shell_physical_owner tl_explicit_nodal_state)
target_compile_features(tl_solid_batch PUBLIC cxx_std_17)
set_target_properties(tl_solid_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_solid_batch PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
