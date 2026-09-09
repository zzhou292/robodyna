# Opt-in P2 parity and separately compiled ANS measurement. The baseline P1
# measurement now includes only host fixture utilities, without parity kernels.
if(NOT TARGET robo_dyna_reissner_prescribed_batch_check)
  include("${CMAKE_CURRENT_LIST_DIR}/PrescribedShellBatchChecks.cmake")
endif()

add_executable(robo_dyna_reissner_ans_rows_check reissner_ans_rows_check.cu)
target_link_libraries(robo_dyna_reissner_ans_rows_check PRIVATE
  tl_reissner_shell robo_dyna_reissner_setup CUDA::cudart GTest::gtest_main)

add_executable(robo_dyna_reissner_prescribed_ans_rows_check reissner_prescribed_batch_check.cu)
target_link_libraries(robo_dyna_reissner_prescribed_ans_rows_check PRIVATE
  tl_prescribed_reissner_ans_rows_batch robo_dyna_reissner_setup GTest::gtest_main)
target_compile_definitions(robo_dyna_reissner_prescribed_ans_rows_check PRIVATE
  ROBO_DYNA_PRESCRIBED_EXPECTED_OPERATION="ans_rows")

foreach(check_target IN ITEMS robo_dyna_reissner_ans_rows_check robo_dyna_reissner_prescribed_ans_rows_check)
  target_compile_definitions(${check_target} PRIVATE EIGEN_NO_CUDA)
  set_target_properties(${check_target} PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(${check_target} PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
endforeach()
add_test(NAME reissner_ans_rows COMMAND robo_dyna_reissner_ans_rows_check)
add_test(NAME reissner_prescribed_ans_rows COMMAND robo_dyna_reissner_prescribed_ans_rows_check)
set_tests_properties(reissner_ans_rows reissner_prescribed_ans_rows PROPERTIES
  TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
