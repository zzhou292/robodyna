include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedSearchPostKinChkValues.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
add_library(robo_dyna_tied_cin_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/tied_cin/Inputs.cpp" "${CMAKE_CURRENT_LIST_DIR}/tied_cin/Budget.cpp")
target_link_libraries(robo_dyna_tied_cin_values PUBLIC robo_dyna_tied_post_kinchk_values tl_tied_cin_attachment)
target_compile_features(robo_dyna_tied_cin_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_cin_values PRIVATE -fno-fast-math -ffp-contract=off)
