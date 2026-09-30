# Included by the owning qualification project; the analytic oracle is header-only.
add_executable(shell_batch_binding_check ShellBatchBindingTest.cpp ShellBatchCollectionTest.cpp)
target_link_libraries(shell_batch_binding_check PRIVATE tl_shell_batch_binding GTest::gtest_main)
target_compile_features(shell_batch_binding_check PRIVATE cxx_std_17)
target_compile_options(shell_batch_binding_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME shell_batch_binding_check COMMAND shell_batch_binding_check)
set_tests_properties(shell_batch_binding_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
