include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_source/VehicleSourcePlan.cmake")
add_library(robo_dyna_tied_shell_declaration STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedShellDeclaration.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceCards.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ScopeDeclarations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/GeometryCensus.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ConstraintEvidence.cpp")
target_link_libraries(robo_dyna_tied_shell_declaration PUBLIC robo_dyna_vehicle_source)
target_compile_features(robo_dyna_tied_shell_declaration PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_shell_declaration PRIVATE -fno-fast-math -ffp-contract=off)
