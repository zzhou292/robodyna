add_executable(type45_resident_cuda OwnerFixture.cu OwnerScatter.cu OwnerNative.cu OwnerTest.cu
  ../physical_publication/OwnerStartup.cu ../physical_publication/OwnerAttempt.cu
  ../physical_publication/OwnerSnapshot.cu)
target_link_libraries(type45_resident_cuda PRIVATE physical_publication_fixture tl_shell_batch_publication
  type45_oracle GTest::gtest_main CUDA::cudart)
set_target_properties(type45_resident_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(type45_resident_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME type45_resident_cuda COMMAND type45_resident_cuda)
set_tests_properties(type45_resident_cuda PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE PROCESSORS 1)
