# One mixed participant gate in the existing T3 qualification project.
add_executable(mixed_shell_batch_check
  MixedShellFixture.cu MixedShellLedger.cu MixedShellStateTest.cu MixedShellFailureTest.cu
  MixedShellFeedbackFixture.cu MixedShellFeedbackTest.cu
  MixedMovingStartupFixture.cu MixedMovingStartupTest.cu)
target_link_libraries(mixed_shell_batch_check PRIVATE
  tl_shell_batch_publication qeph_q1_native t3_r3_native CUDA::cudart GTest::gtest_main)
set_target_properties(mixed_shell_batch_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(mixed_shell_batch_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME mixed_shell_batch_check COMMAND mixed_shell_batch_check)
set_tests_properties(mixed_shell_batch_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
# Larger collections exercise the same joined-publication feature and libraries.
add_subdirectory("../collections" "collections")
