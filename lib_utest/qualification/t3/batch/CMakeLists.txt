# Included by the one owning T3 qualification project after explicit batch/CUDA
# and native force-oracle setup. Not a standalone project or dynamics runner.
add_executable(t3_batch_check T3BatchStateTest.cu T3BatchFailureTest.cu)
target_link_libraries(t3_batch_check PRIVATE tl_t3_batch t3_r3_native CUDA::cudart GTest::gtest_main)
set_target_properties(t3_batch_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(t3_batch_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME t3_batch_check COMMAND t3_batch_check)
set_tests_properties(t3_batch_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
