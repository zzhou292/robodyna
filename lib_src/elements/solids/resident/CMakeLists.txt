include_guard(GLOBAL)
if(NOT TARGET tl_solid_model)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}/model")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../../ShellPhysicalOwnerValues.cmake")
add_library(tl_solid_batch_values STATIC Arena.cpp Plan.cpp Upload.cpp Forecast.cpp Identity.cpp)
target_link_libraries(tl_solid_batch_values PUBLIC tl_solid_model tl_shell_physical_owner_values)
target_compile_features(tl_solid_batch_values PUBLIC cxx_std_17)
target_compile_options(tl_solid_batch_values PRIVATE -fno-fast-math -ffp-contract=off)

option(TL_SOLID_BATCH_CUDA "Build the shared three-family CUDA resident" OFF)
if(TL_SOLID_BATCH_CUDA)
  if(NOT TARGET tl_explicit_nodal_state)
    message(FATAL_ERROR "Solid Batch requires the existing common nodal owner target")
  endif()
  include("${CMAKE_CURRENT_LIST_DIR}/../../ShellPhysicalOwner.cmake")
  add_library(tl_solid_batch STATIC
    Batch.cu Candidate.cu Assembly.cu AssembleOperation.cu EvaluateOperation.cu
    Readback.cu Publication.cpp PublicationScope.cpp OutputRanges.cpp)
  target_link_libraries(tl_solid_batch PUBLIC tl_solid_batch_values
    tl_shell_physical_owner tl_explicit_nodal_state)
  target_compile_features(tl_solid_batch PUBLIC cxx_std_17)
  set_target_properties(tl_solid_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(tl_solid_batch PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
endif()
