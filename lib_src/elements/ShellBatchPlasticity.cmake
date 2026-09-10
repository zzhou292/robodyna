if(NOT TARGET tl_shell_batch_plasticity)
  include("${CMAKE_CURRENT_LIST_DIR}/ShellBatchPlasticityBinding.cmake")
  find_package(CUDAToolkit REQUIRED)
  add_library(tl_shell_batch_plasticity STATIC
    "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPlasticityStorage.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ShellBatchPlasticityCollectionStorage.cpp")
  get_filename_component(shell_batch_plasticity_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
  target_include_directories(tl_shell_batch_plasticity PUBLIC "${shell_batch_plasticity_root}")
  target_compile_features(tl_shell_batch_plasticity PUBLIC cxx_std_17)
  target_link_libraries(tl_shell_batch_plasticity PUBLIC CUDA::cudart tl_shell_batch_plasticity_binding)
  target_compile_options(tl_shell_batch_plasticity PRIVATE -fno-fast-math -ffp-contract=off)
endif()
