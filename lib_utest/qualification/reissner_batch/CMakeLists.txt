# Opt-in prescribed-force qualification library. The composing project enables
# CUDA; no Chrono, dynamics owner or accepted-state target is linked here.
find_package(CUDAToolkit REQUIRED)
if(NOT TARGET tl_reissner_shell)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../../lib_src/elements"
                   "${CMAKE_CURRENT_BINARY_DIR}/reissner-elements")
endif()
# Same bounded wrapper compiled into separate modules. Never link these
# targets into one executable: their public wrapper symbols intentionally match.
foreach(batch_target IN ITEMS tl_prescribed_reissner_batch tl_prescribed_reissner_ans_rows_batch tl_prescribed_reissner_gauss_contract_batch)
  add_library(${batch_target} STATIC PrescribedShellBatch.cu)
  target_include_directories(${batch_target} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(${batch_target} PUBLIC tl_reissner_shell CUDA::cudart)
  set_target_properties(${batch_target} PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(${batch_target} PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
endforeach()
target_compile_definitions(tl_prescribed_reissner_ans_rows_batch PRIVATE TL_PRESCRIBED_FORCE_ANS_ROWS=1)
target_compile_definitions(tl_prescribed_reissner_gauss_contract_batch PRIVATE TL_PRESCRIBED_FORCE_GAUSS_CONTRACT=1)
