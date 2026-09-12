enable_language(CUDA)
set(PHYSICAL_PUBLICATION_CUDA ON CACHE BOOL "Owning mixed physical fixture" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../physical_publication" physical-owner)
add_executable(qeph_mapped_activity_cuda OwnerTest.cu FailureTest.cu KernelTest.cu Transfers.cpp
  FailureKernelTest.cu FailureReadbackTest.cu
  ../physical_publication/OwnerStartup.cu ../physical_publication/OwnerAttempt.cu)
target_link_libraries(qeph_mapped_activity_cuda PRIVATE physical_publication_fixture
  tl_shell_batch_publication CUDA::cudart GTest::gtest_main)
target_link_options(qeph_mapped_activity_cuda PRIVATE "-Wl,--wrap=cudaMemcpyAsync")
set_target_properties(qeph_mapped_activity_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(qeph_mapped_activity_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME qeph_mapped_activity_cuda COMMAND qeph_mapped_activity_cuda)
set_tests_properties(qeph_mapped_activity_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 240)
