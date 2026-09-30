include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/CorrectedNodalSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/CoatedSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25MainGeometry.cmake")
add_library(robo_dyna_selected_shell_main_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SelectedShellMainSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/Binding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/Grouping.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/ShellValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/Certificate.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/SupportQuery.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/SolidSupportQuery.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/Compose.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main_coefficients/Digest.cpp")
target_link_libraries(robo_dyna_selected_shell_main_source PUBLIC
  robo_dyna_vehicle_corrected_nodal_source robo_dyna_v5_coated_source tl_radioss_type25_main_geometry)
target_compile_features(robo_dyna_selected_shell_main_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_selected_shell_main_source PRIVATE -fno-fast-math -ffp-contract=off)
