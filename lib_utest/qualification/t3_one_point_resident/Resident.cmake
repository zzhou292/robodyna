add_executable(t3_one_point_resident_cuda_test
  "${CMAKE_CURRENT_LIST_DIR}/ResidentStartup.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentFrames.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ReadbackTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ReadFault.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../t3_one_point/NativeOracle.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../mixed_layered_resident/MixedResidentFrames.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../t3/mixed/MixedShellFixture.cu")
target_link_libraries(t3_one_point_resident_cuda_test PRIVATE tl_shell_batch_publication
  t3_one_point_native qeph_q1_native t3_r3_native CUDA::cudart GTest::gtest_main)
target_link_options(t3_one_point_resident_cuda_test PRIVATE "-Wl,--wrap=cudaMemcpyAsync")
set_target_properties(t3_one_point_resident_cuda_test PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(t3_one_point_resident_cuda_test PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME t3_one_point_resident_cuda COMMAND t3_one_point_resident_cuda_test)
set_tests_properties(t3_one_point_resident_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
