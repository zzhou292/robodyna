include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/EnvelopeOwnerSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_self_contact/native/PostGapmMainSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25SearchStartup.cmake")
add_library(robo_dyna_finite_wall_contact_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/FiniteWallContactSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact/Binding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact/Controls.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact/Values.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact/Packing.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact/Topology.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact/Gaps.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact/Digest.cpp")
target_link_libraries(robo_dyna_finite_wall_contact_source PUBLIC
  robo_dyna_envelope_owner_source robo_dyna_post_gapm_main_source tl_radioss_type25_search_startup)
target_compile_features(robo_dyna_finite_wall_contact_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_finite_wall_contact_source PRIVATE -fno-fast-math -ffp-contract=off)
