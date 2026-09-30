add_executable(solid_candidate_validation_cuda CandidateTest.cu ValidationTest.cu
  CurrentCandidate.cu FrozenCandidate.cu
  "${TL_ROOT}/lib_src/elements/solids/resident/ResultValidation.cu")
target_include_directories(solid_candidate_validation_cuda PRIVATE "${oracle}")
target_link_libraries(solid_candidate_validation_cuda PRIVATE tl_solid_batch_values GTest::gtest_main)
set_target_properties(solid_candidate_validation_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(solid_candidate_validation_cuda PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
add_test(NAME solid_candidate_validation_cuda COMMAND solid_candidate_validation_cuda)
set_tests_properties(solid_candidate_validation_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
