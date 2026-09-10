include_guard(GLOBAL)
if(NOT TARGET robo_dyna_source_shell_collection OR NOT TARGET tl_explicit_nodal_state)
  message(FATAL_ERROR "Source wall contact requires the native source collection and TL nodal owner")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/SourcePartContactGeometry.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../PlacedCanonicalWall.cmake")
include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/collision/NodalWallContactDevice.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../wall_penalty/WallPenaltyCertification.cmake")
add_library(robo_dyna_source_part_wall_setup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartWallSetup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartWallCertification.cpp")
target_link_libraries(robo_dyna_source_part_wall_setup PUBLIC
  robo_dyna_source_part_contact_geometry robo_dyna_placed_canonical_wall
  robo_dyna_source_shell_collection robo_dyna_wall_penalty_certification CUDA::cudart)
target_compile_features(robo_dyna_source_part_wall_setup PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_part_wall_setup PRIVATE -fno-fast-math -ffp-contract=off)
add_library(robo_dyna_source_part_wall_contact STATIC "${CMAKE_CURRENT_LIST_DIR}/SourcePartWallContact.cpp")
target_link_libraries(robo_dyna_source_part_wall_contact PUBLIC
  robo_dyna_source_part_wall_setup tl_nodal_wall_contact_device)
target_compile_options(robo_dyna_source_part_wall_contact PRIVATE -fno-fast-math -ffp-contract=off)
