get_filename_component(host_shell_tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${host_shell_tl_root}/lib_src/elements/ShellBatchPlasticityBinding.cmake")
add_executable(host_shell_collection_check
  "${CMAKE_CURRENT_LIST_DIR}/HostShellBindingTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/HostShellCatalogTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/AllocationFailure.cpp")
target_include_directories(host_shell_collection_check PRIVATE "${host_shell_tl_root}")
target_link_libraries(host_shell_collection_check PRIVATE tl_shell_batch_plasticity_binding GTest::gtest_main)
target_compile_options(host_shell_collection_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME host_shell_collection_check COMMAND host_shell_collection_check)
set_tests_properties(host_shell_collection_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
