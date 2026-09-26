include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/MixedInterfaceSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/ContactGapOperands.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SelectedShellMainSource.cmake")
foreach(capability main_geometry/Reader.h coefficients/ReaderSolidMain.h startup/PostGapmTypes.h)
  if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/collision/radioss_type25/${capability}")
    message(FATAL_ERROR "Post-GAPM source requires reader geometry/coefficient and typed post-support topology capabilities")
  endif()
endforeach()
add_library(robo_dyna_post_gapm_main_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/PostGapmMainSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_main/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_main/Controls.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_main/Support.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_main/CoefficientValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_main/Gaps.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_main/Digest.cpp")
target_link_libraries(robo_dyna_post_gapm_main_source PUBLIC robo_dyna_mixed_interface_source
  robo_dyna_contact_gap_operands robo_dyna_selected_shell_main_source)
target_compile_features(robo_dyna_post_gapm_main_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_post_gapm_main_source PRIVATE -fno-fast-math -ffp-contract=off)
