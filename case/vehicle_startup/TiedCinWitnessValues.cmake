include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedCinAttachmentValues.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
add_library(robo_dyna_tied_cin_witness_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/tied_cin_witness/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_cin_witness/Incidence.cpp")
target_link_libraries(robo_dyna_tied_cin_witness_values PUBLIC robo_dyna_tied_cin_values tl_shell_batch_binding)
target_compile_features(robo_dyna_tied_cin_witness_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_cin_witness_values PRIVATE -fno-fast-math -ffp-contract=off)
