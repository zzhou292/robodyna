add_executable(resident_shell_collection_check
  ResidentCollectionFixture.cu ResidentCollectionOracle.cu ResidentCollectionTest.cu)
target_link_libraries(resident_shell_collection_check PRIVATE
  tl_shell_batch_publication qeph_q1_native t3_r3_native CUDA::cudart GTest::gtest_main)
set_target_properties(resident_shell_collection_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(resident_shell_collection_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME resident_shell_collection_check COMMAND resident_shell_collection_check)
set_tests_properties(resident_shell_collection_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)

# Reuse the same native reference/recurrence fixture with a triangle-heavy mesh.
add_executable(resident_triangle_tail_check
  ResidentCollectionFixture.cu ResidentCollectionOracle.cu ResidentTriangleTailTest.cu)
target_compile_definitions(resident_triangle_tail_check PRIVATE TL_RESIDENT_TRIANGLE_TAIL_FIXTURE)
target_link_libraries(resident_triangle_tail_check PRIVATE
  tl_shell_batch_publication qeph_q1_native t3_r3_native CUDA::cudart GTest::gtest_main)
set_target_properties(resident_triangle_tail_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(resident_triangle_tail_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME resident_triangle_tail_check COMMAND resident_triangle_tail_check)
set_tests_properties(resident_triangle_tail_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
