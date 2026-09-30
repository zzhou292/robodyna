include_guard(GLOBAL)
get_filename_component(nodal_domain_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${nodal_domain_root}/lib_src/assembly/ShellNodeMap.cmake")
add_executable(nodal_domain_check
  "${CMAKE_CURRENT_LIST_DIR}/DomainTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MapTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SizeTest.cpp")
target_link_libraries(nodal_domain_check PRIVATE tl_shell_node_map GTest::gtest_main)
target_compile_options(nodal_domain_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME nodal_domain_values COMMAND nodal_domain_check --gtest_filter=-NodalDomainSize.*)
add_test(NAME nodal_domain_full_count COMMAND nodal_domain_check --gtest_filter=NodalDomainSize.DeclaredMaximum)
add_test(NAME shell_node_map_full_count COMMAND nodal_domain_check --gtest_filter=NodalDomainSize.SourceCountShellMap)
set_tests_properties(nodal_domain_values nodal_domain_full_count PROPERTIES PROCESSORS 1 RUN_SERIAL TRUE TIMEOUT 60)
set_tests_properties(shell_node_map_full_count PROPERTIES PROCESSORS 1 RUN_SERIAL TRUE TIMEOUT 180)
