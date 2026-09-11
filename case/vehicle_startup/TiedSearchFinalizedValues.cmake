include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedSearchAssessmentValues.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/tied_shell/search/TiedSearchFinalization.cmake")
add_library(robo_dyna_tied_finalization_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/tied_finalization/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_finalization/Inputs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_finalization/SourceReceipt.cpp")
target_link_libraries(robo_dyna_tied_finalization_values PUBLIC robo_dyna_tied_assessment_values tl_tied_search_finalization)
target_compile_features(robo_dyna_tied_finalization_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_finalization_values PRIVATE -fno-fast-math -ffp-contract=off)
