include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../modelio/source_assembly/SourceAssembly.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../source_assembly_pilot/Metrics.cmake")
if(NOT TARGET shell_layered_native_reference)
  message(FATAL_ERROR "Source spin replay requires the owning native layered reference target")
endif()
add_library(robo_dyna_source_assembly_spin STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceContext.cpp" "${CMAKE_CURRENT_LIST_DIR}/PacketInput.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PacketComparison.cpp" "${CMAKE_CURRENT_LIST_DIR}/LocalDiagnostic.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Analysis.cpp")
target_link_libraries(robo_dyna_source_assembly_spin PUBLIC robo_dyna_source_assembly_shell_input
  robo_dyna_source_assembly_material_input robo_dyna_comparison_metrics shell_layered_native_reference)
target_compile_features(robo_dyna_source_assembly_spin PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_spin PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_assembly_spin_analyze "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries(robo_dyna_source_assembly_spin_analyze PRIVATE robo_dyna_source_assembly_spin)
