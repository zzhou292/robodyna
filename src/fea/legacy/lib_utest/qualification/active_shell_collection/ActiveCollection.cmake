include_guard(GLOBAL)
# Existing integration builds and the standalone package share these owners.
add_executable(active_shell_layout_check "${CMAKE_CURRENT_LIST_DIR}/ArenaLayoutTest.cpp")
target_link_libraries(active_shell_layout_check PRIVATE tl_shell_batch_publication GTest::gtest_main)
target_compile_options(active_shell_layout_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME active_shell_layout_check COMMAND active_shell_layout_check)
set_tests_properties(active_shell_layout_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)

add_executable(active_shell_collection_check
  "${CMAKE_CURRENT_LIST_DIR}/ActiveCollectionFixture.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ActiveTailCheck.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ActiveCollectionTest.cu")
target_link_libraries(active_shell_collection_check PRIVATE tl_shell_batch_publication CUDA::cudart GTest::gtest_main)
set_target_properties(active_shell_collection_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(active_shell_collection_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME active_shell_collection_check COMMAND active_shell_collection_check)
set_tests_properties(active_shell_collection_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
