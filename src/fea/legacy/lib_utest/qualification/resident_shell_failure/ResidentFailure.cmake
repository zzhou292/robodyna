add_executable(resident_shell_failure_cuda_check
  "${CMAKE_CURRENT_LIST_DIR}/FailureResidentStartup.cu"
  "${CMAKE_CURRENT_LIST_DIR}/FailureResidentFrames.cu"
  "${CMAKE_CURRENT_LIST_DIR}/FailureResidentOracle.cu"
  "${CMAKE_CURRENT_LIST_DIR}/FailureResidentTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/FailureReadbackTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/FailureReadFault.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../mixed_layered_resident/MixedResidentFrames.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../resident_plasticity/ResidentPlasticityFixture.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../resident_plasticity/ResidentCollectionFixture.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../t3/mixed/MixedShellFixture.cu")
target_include_directories(resident_shell_failure_cuda_check PRIVATE "${tl_root}")
target_link_libraries(resident_shell_failure_cuda_check PRIVATE tl_shell_batch_publication
  qeph_q1_native t3_r3_native CUDA::cudart GTest::gtest_main)
target_link_options(resident_shell_failure_cuda_check PRIVATE "-Wl,--wrap=cudaMemcpyAsync")
set_target_properties(resident_shell_failure_cuda_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(resident_shell_failure_cuda_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME resident_shell_failure_cuda COMMAND resident_shell_failure_cuda_check)
set_tests_properties(resident_shell_failure_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
