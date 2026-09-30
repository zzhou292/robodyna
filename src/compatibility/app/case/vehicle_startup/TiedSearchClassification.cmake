include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedSearchFinalized.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/TiedSearchClassificationValues.cmake")
add_library(robo_dyna_tied_search_classification STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedSearchClassification.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tied_classification/Budget.cpp")
target_link_libraries(robo_dyna_tied_search_classification PUBLIC
  robo_dyna_tied_search_finalized robo_dyna_tied_classification_values)
target_compile_features(robo_dyna_tied_search_classification PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_search_classification PRIVATE -fno-fast-math -ffp-contract=off)
