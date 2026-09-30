# Included only by the explicit QEPH qualification build after it supplies the
# production owner/batch and native reference targets. Never a runtime donor.
add_executable(qeph_batch_check QephBatchStateTest.cu QephBatchFailureTest.cu)
target_link_libraries(qeph_batch_check PRIVATE tl_qeph_batch qeph_q1_native CUDA::cudart GTest::gtest_main)
set_target_properties(qeph_batch_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(qeph_batch_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME qeph_batch_check COMMAND qeph_batch_check)
set_tests_properties(qeph_batch_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
