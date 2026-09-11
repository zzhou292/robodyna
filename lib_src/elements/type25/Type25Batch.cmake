# Optional resident joined TYPE25 contributor. The composing build enables
# CUDA and supplies tl_explicit_nodal_state. No native/reference dependency.
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/Type25Math.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../assembly/NodalMassBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ShellPhysicalOwner.cmake")
add_library(tl_type25_batch STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Type25Batch.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Type25BatchOperations.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Type25BatchKernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Type25BatchReadback.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Type25BatchStartup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Type25BatchArena.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Type25BatchIdentity.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Type25BatchPublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Startup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Initialize.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Assemble.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Kernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/mapped/Publication.cpp")
target_link_libraries(tl_type25_batch PUBLIC tl_type25_math tl_nodal_mass_binding tl_explicit_nodal_state tl_shell_physical_owner)
target_compile_features(tl_type25_batch PUBLIC cxx_std_17)
set_target_properties(tl_type25_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_type25_batch PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
