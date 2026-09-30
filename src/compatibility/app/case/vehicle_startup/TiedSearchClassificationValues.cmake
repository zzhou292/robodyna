include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedSearchFinalizedValues.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../modelio/tied_shell/TiedClassificationContext.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/tied_shell/TiedClassification.cmake")
add_library(robo_dyna_tied_classification_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/tied_classification/Inputs.cpp")
target_link_libraries(robo_dyna_tied_classification_values PUBLIC
  robo_dyna_tied_finalization_values robo_dyna_tied_classification_context tl_tied_shell_classification)
target_compile_features(robo_dyna_tied_classification_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_classification_values PRIVATE -fno-fast-math -ffp-contract=off)
