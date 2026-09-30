include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../tied_shell/TiedShellDeclaration.cmake")

add_library(robo_dyna_original_self_contact_selection STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Sources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Cards.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PartSets.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Census.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/OriginalSelection.cpp")
target_link_libraries(robo_dyna_original_self_contact_selection
  PUBLIC robo_dyna_tied_shell_declaration)
target_compile_features(robo_dyna_original_self_contact_selection
  PUBLIC cxx_std_17)
target_compile_options(robo_dyna_original_self_contact_selection
  PRIVATE -fno-fast-math -ffp-contract=off)
