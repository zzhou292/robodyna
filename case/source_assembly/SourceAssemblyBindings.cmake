include_guard(GLOBAL)
get_filename_component(robo_assembly_case_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
# Reuse modelio's owned source/adapters; case owns composition, TL owns mechanics.
include("${robo_assembly_case_root}/modelio/source_assembly/SourceAssembly.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchPlasticityBinding.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/NodalRigidGroup.cmake")
add_library(robo_dyna_source_assembly_bindings STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyBindings.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyPartLedger.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyRigidStartup.cpp")
target_link_libraries(robo_dyna_source_assembly_bindings PUBLIC
  robo_dyna_source_assembly_shell_input robo_dyna_source_assembly_material_input
  tl_shell_batch_plasticity_binding tl_nodal_rigid_group)
target_compile_features(robo_dyna_source_assembly_bindings PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_bindings PRIVATE -fno-fast-math -ffp-contract=off)
