# Opt-in P1 qualification. Reuse the actual Chrono reference/setup; this adds
# no production dynamics owner and never schedules an automatic size ladder.
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)
find_package(GTest REQUIRED)
if(NOT TARGET robo_dyna_reissner_setup)
  include("${CMAKE_CURRENT_LIST_DIR}/ReissnerSetup.cmake")
endif()
add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/qualification/reissner_batch"
                 "tl-prescribed-reissner-batch")
add_executable(robo_dyna_reissner_prescribed_batch_check reissner_prescribed_batch_check.cu)
target_link_libraries(robo_dyna_reissner_prescribed_batch_check PRIVATE
  tl_prescribed_reissner_batch robo_dyna_reissner_setup GTest::gtest_main)
target_compile_definitions(robo_dyna_reissner_prescribed_batch_check PRIVATE EIGEN_NO_CUDA)
set_target_properties(robo_dyna_reissner_prescribed_batch_check PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo_dyna_reissner_prescribed_batch_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME reissner_prescribed_batch COMMAND robo_dyna_reissner_prescribed_batch_check)
set_tests_properties(reissner_prescribed_batch PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
