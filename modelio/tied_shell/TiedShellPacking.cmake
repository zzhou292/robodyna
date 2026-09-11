include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedShellDeclaration.cmake")
add_library(robo_dyna_tied_shell_packing STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedShellPacking.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/packing/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/packing/Policy.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/packing/Order.cpp")
target_link_libraries(robo_dyna_tied_shell_packing PUBLIC robo_dyna_tied_shell_declaration)
target_compile_features(robo_dyna_tied_shell_packing PUBLIC cxx_std_17)
