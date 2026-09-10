include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../case/source_part_wall/SourcePartContactGeometry.cmake")
add_library(robo_dyna_source_nodal_wall_fixture STATIC "${CMAKE_CURRENT_LIST_DIR}/SourceNodalWallFixture.cpp")
target_link_libraries(robo_dyna_source_nodal_wall_fixture PUBLIC
  robo_dyna_source_contact_force_fixture robo_dyna_source_part_contact_geometry tl_nodal_wall_contact)
target_compile_options(robo_dyna_source_nodal_wall_fixture PRIVATE -fno-fast-math -ffp-contract=off)
