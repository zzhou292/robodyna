include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../constraints/NodalRigidAssemblyBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/ShellBatchPlasticityBinding.cmake")
add_library(tl_shell_execution_binding STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ShellExecutionBinding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/shell_execution/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/shell_execution/Parents.cpp")
target_link_libraries(tl_shell_execution_binding PUBLIC
  tl_nodal_rigid_assembly_binding tl_shell_batch_plasticity_binding)
target_compile_features(tl_shell_execution_binding PUBLIC cxx_std_17)
target_compile_options(tl_shell_execution_binding PRIVATE -fno-fast-math -ffp-contract=off)
