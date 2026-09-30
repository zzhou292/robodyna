include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../tied_shell/TiedShellDeclaration.cmake")
add_library(robo_dyna_beam18_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Source.cpp" "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Declarations.cpp" "${CMAKE_CURRENT_LIST_DIR}/Part.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Geometry.cpp" "${CMAKE_CURRENT_LIST_DIR}/WorkingCards.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/References.cpp")
target_link_libraries(robo_dyna_beam18_source PUBLIC robo_dyna_tied_shell_declaration)
target_compile_features(robo_dyna_beam18_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_beam18_source PRIVATE -fno-fast-math -ffp-contract=off)
