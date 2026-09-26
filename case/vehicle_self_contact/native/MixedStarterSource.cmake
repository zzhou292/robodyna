include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/PostGapmMainSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25CurrentNormals.cmake")
add_library(robo_dyna_mixed_starter_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/MixedStarterSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_starter/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_starter/Domain.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_starter/Digest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_starter/Document.cpp")
target_link_libraries(robo_dyna_mixed_starter_source PUBLIC
  robo_dyna_post_gapm_main_source tl_radioss_type25_current_normals)
target_compile_features(robo_dyna_mixed_starter_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_mixed_starter_source PRIVATE -fno-fast-math -ffp-contract=off)
