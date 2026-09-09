# Explicit qualification subdirectory only; native Fortran is a test oracle.
add_executable(qeph_coupled_check QephCoupledFixture.cu QephCoupledNative.cu
  QephCoupledLedger.cu QephCoupledTest.cu)
target_link_libraries(qeph_coupled_check PRIVATE tl_qeph_batch qeph_q1_native CUDA::cudart GTest::gtest_main)
set_target_properties(qeph_coupled_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(qeph_coupled_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME qeph_coupled_check COMMAND qeph_coupled_check)
set_tests_properties(qeph_coupled_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
