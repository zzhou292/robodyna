include_guard(GLOBAL)
if(NOT TARGET tl_nodal_wall_contact)
  message(FATAL_ERROR "Wall penalty value certification requires the owning TL nodal wall types")
endif()
add_library(robo_dyna_wall_penalty_certification STATIC
  "${CMAKE_CURRENT_LIST_DIR}/UniformTranslationKinetic.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/WallPenaltyCertification.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/WallPlacementBounds.cpp")
get_filename_component(robo_wall_penalty_app_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(robo_dyna_wall_penalty_certification PUBLIC "${robo_wall_penalty_app_root}")
target_link_libraries(robo_dyna_wall_penalty_certification PUBLIC tl_nodal_wall_contact)
target_compile_features(robo_dyna_wall_penalty_certification PUBLIC cxx_std_17)
target_compile_options(robo_dyna_wall_penalty_certification PRIVATE -fno-fast-math -ffp-contract=off)
