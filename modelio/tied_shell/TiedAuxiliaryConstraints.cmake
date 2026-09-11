include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedShellDeclaration.cmake")
add_library(robo_dyna_tied_auxiliary_constraints STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedAuxiliaryConstraints.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/auxiliary/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/auxiliary/Sources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/auxiliary/Groups.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/auxiliary/Nodes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/auxiliary/Boundary.cpp")
target_link_libraries(robo_dyna_tied_auxiliary_constraints PUBLIC robo_dyna_tied_shell_declaration)
target_compile_features(robo_dyna_tied_auxiliary_constraints PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_auxiliary_constraints PRIVATE -fno-fast-math -ffp-contract=off)
