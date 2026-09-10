include_guard(GLOBAL)
get_filename_component(robo_assembly_output_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${robo_assembly_output_root}/modelio/source_assembly/SourceAssembly.cmake")
include("${robo_assembly_output_root}/chrono/AcceptedSurface.cmake")
add_library(robo_dyna_source_assembly_fields STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblySurface.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblySectionFields.cpp")
target_link_libraries(robo_dyna_source_assembly_fields PUBLIC robo_dyna_source_assembly crash_accepted_surface)
target_compile_options(robo_dyna_source_assembly_fields PRIVATE -fno-fast-math -ffp-contract=off)
