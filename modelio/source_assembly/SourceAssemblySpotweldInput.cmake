include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssembly.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/type25/Type25Model.cmake")
add_library(robo_dyna_source_assembly_spotweld_input STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblySpotweldInput.cpp")
target_link_libraries(robo_dyna_source_assembly_spotweld_input PUBLIC
  robo_dyna_source_assembly tl_type25_model)
target_compile_options(robo_dyna_source_assembly_spotweld_input PRIVATE -fno-fast-math -ffp-contract=off)
