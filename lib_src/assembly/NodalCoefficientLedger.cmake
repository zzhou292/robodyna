include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/ShellNodeMap.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/Type13NodeContributions.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/ElementMassContributions.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SolidNodeContributions.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/type25/Type25Model.cmake")
add_library(tl_nodal_coefficient_ledger STATIC
  "${CMAKE_CURRENT_LIST_DIR}/NodalCoefficientLedger.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalCoefficientBudget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalCoefficientChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalCoefficientShells.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalCoefficientSprings.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalCoefficientPointMasses.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalCoefficientSolids.cpp")
target_link_libraries(tl_nodal_coefficient_ledger PUBLIC
  tl_shell_node_map tl_type13_node_contributions tl_type25_model tl_element_mass_contributions
  tl_solid_node_contributions)
target_compile_features(tl_nodal_coefficient_ledger PUBLIC cxx_std_17)
target_compile_options(tl_nodal_coefficient_ledger PRIVATE -fno-fast-math -ffp-contract=off)
