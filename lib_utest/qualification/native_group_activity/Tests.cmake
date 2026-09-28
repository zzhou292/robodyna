add_executable(native_group_activity_host "${CMAKE_CURRENT_LIST_DIR}/HostTest.cpp")
target_link_libraries(native_group_activity_host PRIVATE tl_radioss_type25_transaction GTest::gtest_main)
target_compile_options(native_group_activity_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME native_group_activity_host COMMAND native_group_activity_host)
add_executable(native_group_activity_cuda "${CMAKE_CURRENT_LIST_DIR}/CudaTest.cu" "${CMAKE_CURRENT_LIST_DIR}/Probe.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../radioss_type25_runtime/FullLedgerRig.cu")
target_link_libraries(native_group_activity_cuda PRIVATE tl_radioss_type25_transaction GTest::gtest_main CUDA::cudart)
set_target_properties(native_group_activity_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(native_group_activity_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
target_link_options(native_group_activity_cuda PRIVATE -Wl,--wrap=cudaMemcpyAsync -Wl,--wrap=cudaStreamSynchronize)
add_test(NAME native_group_activity_cuda COMMAND native_group_activity_cuda)
set_tests_properties(native_group_activity_host native_group_activity_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
