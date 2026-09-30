enable_language(CUDA)
set(PHYSICAL_PUBLICATION_CUDA ON CACHE BOOL "Owning mixed physical fixture" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../physical_publication" physical-owner)
add_executable(t3_activity_readback_cuda OwnerTest.cu FailureTest.cu PreflightTest.cu Transfers.cpp
  ../physical_publication/OwnerStartup.cu ../physical_publication/OwnerAttempt.cu)
target_link_libraries(t3_activity_readback_cuda PRIVATE physical_publication_fixture
  tl_shell_batch_publication CUDA::cudart GTest::gtest_main)
target_link_options(t3_activity_readback_cuda PRIVATE
  "-Wl,--wrap=cudaMemcpyAsync" "-Wl,--wrap=cudaStreamSynchronize")
set_target_properties(t3_activity_readback_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(t3_activity_readback_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME t3_activity_readback_cuda COMMAND t3_activity_readback_cuda)
set_tests_properties(t3_activity_readback_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 240)
