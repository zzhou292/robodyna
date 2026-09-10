include_guard(GLOBAL)
get_filename_component(robo_assembly_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "TL-FEA source checkout supplying typed startup inputs")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchBinding.h" OR
   NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchPlasticityBinding.h")
  message(FATAL_ERROR "Assembly input adapters require an explicit TL-FEA source checkout with shell/material input declarations")
endif()
include("${robo_assembly_root}/output/ArtifactIO.cmake")
add_library(robo_dyna_source_assembly STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssembly.cpp" "${CMAKE_CURRENT_LIST_DIR}/JsonReader.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadDeclarations.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReadGeometry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadAttachments.cpp")
target_include_directories(robo_dyna_source_assembly PUBLIC "${robo_assembly_root}" "${ROBO_DYNA_TL_ROOT}")
target_link_libraries(robo_dyna_source_assembly PUBLIC robo_dyna_artifact_io)
target_compile_features(robo_dyna_source_assembly PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly PRIVATE -fno-fast-math -ffp-contract=off)
foreach(kind Shell Material)
  string(TOLOWER "${kind}" lower_kind)
  add_library(robo_dyna_source_assembly_${lower_kind}_input STATIC "${CMAKE_CURRENT_LIST_DIR}/SourceAssembly${kind}Input.cpp")
  target_link_libraries(robo_dyna_source_assembly_${lower_kind}_input PUBLIC robo_dyna_source_assembly)
  target_compile_options(robo_dyna_source_assembly_${lower_kind}_input PRIVATE -fno-fast-math -ffp-contract=off)
endforeach()
