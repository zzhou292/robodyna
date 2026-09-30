include_guard(GLOBAL)
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/collision/NodalWallContact.cmake")
add_library(robo_dyna_source_part_contact_geometry STATIC "${CMAKE_CURRENT_LIST_DIR}/SourcePartContactGeometry.cpp")
target_link_libraries(robo_dyna_source_part_contact_geometry PUBLIC
  robo_dyna_source_contact_fixture tl_nodal_wall_contact tl_q4_parametric_contact)
target_compile_options(robo_dyna_source_part_contact_geometry PRIVATE -fno-fast-math -ffp-contract=off)
