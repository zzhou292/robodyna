# Explicit qualification subdirectory only; native Fortran is a test oracle.
add_library(qeph_coupled_fixture STATIC QephCoupledFixture.cu QephCoupledNative.cu QephCoupledLedger.cu)
target_link_libraries(qeph_coupled_fixture PUBLIC tl_qeph_batch qeph_q1_native CUDA::cudart GTest::gtest)
set_target_properties(qeph_coupled_fixture PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(qeph_coupled_fixture PUBLIC
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_executable(qeph_coupled_check QephCoupledTest.cu)
target_link_libraries(qeph_coupled_check PRIVATE qeph_coupled_fixture GTest::gtest_main)
set_target_properties(qeph_coupled_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
add_test(NAME qeph_coupled_check COMMAND qeph_coupled_check)
set_tests_properties(qeph_coupled_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

# Explicit incoming-velocity startup uses the same owner/native fixture.
include("${TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
add_executable(qeph_uniform_startup_check ../batch/QephUniformStartupTest.cu)
target_link_libraries(qeph_uniform_startup_check PRIVATE
  qeph_coupled_fixture tl_shell_batch_binding GTest::gtest_main)
set_target_properties(qeph_uniform_startup_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
add_test(NAME qeph_uniform_startup_check COMMAND qeph_uniform_startup_check)
set_tests_properties(qeph_uniform_startup_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
