# Closed mixed-shell publication; the composing project provides both typed
# batches and the sole nodal owner. Native oracles are test-only dependencies.
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/type25/Type25Batch.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/qbat/QbatBatch.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/type13/resident/Batch.cmake")
if(NOT TARGET tl_solid_batch_values)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/solids/resident" "${CMAKE_CURRENT_BINARY_DIR}/physical-solid-resident")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/solids/resident/Batch.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/type45/resident/Batch.cmake")
if(NOT TARGET tl_beam18_batch_values)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/beam18/resident" "${CMAKE_CURRENT_BINARY_DIR}/physical-beam18-resident")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/beam18/resident/Batch.cmake")
add_library(tl_shell_batch_publication STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/publication/PhysicalValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/publication/PhysicalForecast.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/publication/PhysicalStartup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/publication/PhysicalPreflight.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/publication/PhysicalTransaction.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/publication/PhysicalReadback.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/publication/ShellPhysicalScratchParticipation.cpp"

  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPublicationValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchFormulationStartup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchFormulationPreflight.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchFormulationPublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPublicationKernels.cu")
target_link_libraries(tl_shell_batch_publication PUBLIC
  tl_qeph_batch tl_t3_batch tl_type25_batch tl_type13_batch tl_solid_batch tl_type45_batch tl_beam18_batch tl_qbat_batch tl_shell_batch_binding tl_explicit_nodal_state)
set_target_properties(tl_shell_batch_publication PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_shell_batch_publication PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
