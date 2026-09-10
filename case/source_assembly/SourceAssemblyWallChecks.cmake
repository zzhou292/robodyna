# Optional actual finite-wall host gate; no CUDA state or trajectory.
set(ROBO_DYNA_SOURCE_ASSEMBLY_WALL "" CACHE FILEPATH "Frozen finite wall manifest for assembly coverage checks")
if(ROBO_DYNA_SOURCE_ASSEMBLY_WALL)
  if(NOT EXISTS "${ROBO_DYNA_SOURCE_ASSEMBLY_WALL}")
    message(FATAL_ERROR "Declared source assembly wall manifest must exist")
  endif()
  include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/Q4PlanarGeometry.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/../CanonicalWallArtifacts.cmake")
  # Existing wall adapters use this established TL include-root name.
  set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
  include("${CMAKE_CURRENT_LIST_DIR}/../PlacedCanonicalWall.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallSetup.cmake")
  add_executable(robo_dyna_source_assembly_wall_check tests/SourceAssemblyWallTest.cpp
    tests/SourceAssemblyWallSetupTest.cpp tests/SourceAssemblyWallConfigTest.cpp
    "${CMAKE_CURRENT_LIST_DIR}/../wall_penalty/WallPenaltyValueTest.cpp")
  target_link_libraries(robo_dyna_source_assembly_wall_check PRIVATE robo_dyna_source_assembly_bindings
    robo_dyna_shell_collection_contact_geometry robo_dyna_placed_canonical_wall
    robo_dyna_canonical_wall_artifacts robo_dyna_source_assembly_wall_setup GTest::gtest_main)
  target_compile_options(robo_dyna_source_assembly_wall_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_assembly_wall COMMAND robo_dyna_source_assembly_wall_check)
  set_tests_properties(source_assembly_wall PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120
    ENVIRONMENT "ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY};ROBO_DYNA_SOURCE_ASSEMBLY_WALL=${ROBO_DYNA_SOURCE_ASSEMBLY_WALL}")
endif()
