add_executable(qbat_resident_cuda_test
  "${CMAKE_CURRENT_LIST_DIR}/ResidentStartup.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentFrames.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ReadbackTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ConnectorTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/OriginalTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ReadFault.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../qbat_force/NativeOracle.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../qbat_binding/OriginalFixture.cpp")
target_link_libraries(qbat_resident_cuda_test PRIVATE tl_shell_batch_publication qbat_force_native CUDA::cudart GTest::gtest_main)
target_link_options(qbat_resident_cuda_test PRIVATE "-Wl,--wrap=cudaMemcpyAsync")
set_target_properties(qbat_resident_cuda_test PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(qbat_resident_cuda_test PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME qbat_resident_cuda COMMAND qbat_resident_cuda_test --gtest_filter=-QbatResidentCuda.Original4250*)
add_test(NAME qbat_resident_original_cuda COMMAND qbat_resident_cuda_test --gtest_filter=QbatResidentCuda.Original4250*)
set_tests_properties(qbat_resident_original_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 240)
