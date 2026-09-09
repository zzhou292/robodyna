# Opt-in prescribed-force qualification library. The composing project enables
# CUDA; no Chrono, dynamics owner or accepted-state target is linked here.
find_package(CUDAToolkit REQUIRED)
if(NOT TARGET tl_reissner_shell)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../../lib_src/elements"
                   "${CMAKE_CURRENT_BINARY_DIR}/reissner-elements")
endif()
add_library(tl_prescribed_reissner_batch STATIC PrescribedShellBatch.cu)
target_include_directories(tl_prescribed_reissner_batch PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(tl_prescribed_reissner_batch PUBLIC tl_reissner_shell CUDA::cudart)
set_target_properties(tl_prescribed_reissner_batch PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_prescribed_reissner_batch PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
