include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/PostGapmMainSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_wall/native/WallSource.cmake")
add_library(robo_dyna_initializer_controls_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/InitializerControlsSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/initial_controls/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/initial_controls/Controls.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/initial_controls/Gaps.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/initial_controls/Law.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/initial_controls/Namespace.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/initial_controls/Digest.cpp")
target_link_libraries(robo_dyna_initializer_controls_source PUBLIC
  robo_dyna_post_gapm_main_source robo_dyna_native_envelope_wall_source)
target_compile_features(robo_dyna_initializer_controls_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_initializer_controls_source PRIVATE -fno-fast-math -ffp-contract=off)
