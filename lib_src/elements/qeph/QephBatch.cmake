# Optional resident QEPH participant; composing build enables CUDA and supplies
# tl_qeph and tl_explicit_nodal_state. No native/reference/test dependency.
include("${CMAKE_CURRENT_LIST_DIR}/../ShellBatchBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ShellBatchPlasticity.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ShellMappedStartup.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ShellPhysicalOwner.cmake")
add_library(tl_qeph_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/rejected_candidate/Copy.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/rejected_candidate/Inputs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/rejected_candidate/Replay.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Startup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Initialize.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Assemble.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Kernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/ObserverKernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Publication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Readback.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/ActivityReadback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/ActivityKernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/FailureActivityReadback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/FailureActivityKernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/MixedActivityReadback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/MixedActivityKernels.cu"
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

include("${CMAKE_CURRENT_LIST_DIR}/../ShellFormulationScope.cmake")
target_link_libraries(tl_qeph_batch PUBLIC tl_shell_formulation_scope)

target_link_libraries(tl_qeph_batch PUBLIC tl_shell_mapped_startup tl_shell_physical_owner)
