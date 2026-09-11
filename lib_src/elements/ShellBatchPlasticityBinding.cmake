if(NOT TARGET tl_shell_batch_plasticity_binding)
  if(NOT TARGET tl_shell_batch_binding)
    include("${CMAKE_CURRENT_LIST_DIR}/ShellBatchBinding.cmake")
  endif()
  add_library(tl_shell_batch_plasticity_binding STATIC
    "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPlasticityBinding.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPlasticityBindingSetup.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ShellBatchSectionBinding.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ShellBatchFormulationBinding.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPlasticityBindingParents.cpp")
  target_link_libraries(tl_shell_batch_plasticity_binding PUBLIC tl_shell_batch_binding)
  target_compile_features(tl_shell_batch_plasticity_binding PUBLIC cxx_std_17)
  target_compile_options(tl_shell_batch_plasticity_binding PRIVATE -fno-fast-math -ffp-contract=off)
endif()
