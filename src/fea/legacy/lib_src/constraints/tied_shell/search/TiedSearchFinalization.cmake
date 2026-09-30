include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../TiedSearch.cmake")
add_library(tl_tied_search_finalization STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedSearchFinalization.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FinalizationChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FinalizationMaps.cpp")
target_link_libraries(tl_tied_search_finalization PUBLIC tl_tied_shell_search)
target_compile_features(tl_tied_search_finalization PUBLIC cxx_std_17)
target_compile_options(tl_tied_search_finalization PRIVATE -fno-fast-math -ffp-contract=off)
