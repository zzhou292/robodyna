include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedSearchClassificationValues.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/tied_shell/TiedPostKinChk.cmake")
add_library(robo_dyna_tied_post_kinchk_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/tied_post_kinchk/Receipt.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_post_kinchk/Inputs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_post_kinchk/Budget.cpp")
target_link_libraries(robo_dyna_tied_post_kinchk_values PUBLIC robo_dyna_tied_classification_values tl_tied_post_kinchk)
target_compile_features(robo_dyna_tied_post_kinchk_values PUBLIC cxx_std_17)
