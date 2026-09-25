add_executable(physical_publication_cuda OwnerStartup.cu OwnerAttempt.cu OwnerSnapshot.cu
  OwnerTest.cu ParticipationTest.cu NativeContactPublicationTest.cu Type13MappedTest.cu)
target_link_libraries(physical_publication_cuda PRIVATE physical_publication_fixture
  tl_shell_batch_publication CUDA::cudart)
set_target_properties(physical_publication_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(physical_publication_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME physical_publication_cuda COMMAND physical_publication_cuda)
set_tests_properties(physical_publication_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 240)
