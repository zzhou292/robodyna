# Immutable authenticated inventory only; no native startup, solver or CUDA targets.
include_guard(GLOBAL)
get_filename_component(robo_assembly_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ROBO_DYNA_TL_ROOT "${CRASH_TL_FEA_SOURCE_DIR}" CACHE PATH "TL-FEA checkout supplying neutral source vector declarations")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/math/Fixed3.h")
  message(FATAL_ERROR "Assembly reader requires explicit ROBO_DYNA_TL_ROOT (neutral Fixed3.h only)")
endif()
include("${robo_assembly_root}/output/ArtifactIO.cmake")
add_library(robo_dyna_source_assembly STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssembly.cpp" "${CMAKE_CURRENT_LIST_DIR}/JsonReader.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadDeclarations.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReadLaw44Material.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadGeometry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadLaw1Material.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReadMaterialPolicy.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadAuxiliaryFrontier.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadAttachments.cpp")
target_include_directories(robo_dyna_source_assembly PUBLIC "${robo_assembly_root}" "${ROBO_DYNA_TL_ROOT}")
target_link_libraries(robo_dyna_source_assembly PUBLIC robo_dyna_artifact_io)
target_compile_features(robo_dyna_source_assembly PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly PRIVATE -fno-fast-math -ffp-contract=off)
