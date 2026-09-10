include_guard(GLOBAL)
if(NOT ROBO_DYNA_SOURCE_SHELL_COLLECTION)
  message(FATAL_ERROR "Source-part wall geometry checks require the native structural shell binding")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/SourcePartContactGeometry.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../PlacedCanonicalWall.cmake")
# Legacy proxy fixture is a test comparison dependency, never a geometry-owner dependency.
include("${CMAKE_CURRENT_LIST_DIR}/../../qualification/source_contact/SourceNodalWallFixtureLibrary.cmake")
add_executable(robo_dyna_source_part_wall_geometry_check "${CMAKE_CURRENT_LIST_DIR}/source_part_wall_geometry_check.cpp")
target_link_libraries(robo_dyna_source_part_wall_geometry_check PRIVATE
  robo_dyna_source_part_contact_geometry robo_dyna_placed_canonical_wall
  robo_dyna_source_nodal_wall_fixture robo_dyna_source_shell_collection GTest::gtest)
target_compile_options(robo_dyna_source_part_wall_geometry_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME source_part_wall_geometry COMMAND robo_dyna_source_part_wall_geometry_check
  "${ROBO_DYNA_SOURCE_PART_READINESS}" "${CRASH_CANONICAL_WALL}")
set_tests_properties(source_part_wall_geometry PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
