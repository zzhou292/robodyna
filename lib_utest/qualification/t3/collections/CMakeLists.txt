add_executable(resident_shell_collection_check
  ResidentCollectionFixture.cu ResidentCollectionOracle.cu ResidentCollectionTest.cu)
target_link_libraries(resident_shell_collection_check PRIVATE
  tl_shell_batch_publication qeph_q1_native t3_r3_native CUDA::cudart GTest::gtest_main)
set_target_properties(resident_shell_collection_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(resident_shell_collection_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME resident_shell_collection_check COMMAND resident_shell_collection_check)
set_tests_properties(resident_shell_collection_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
