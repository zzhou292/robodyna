if(NOT TYPE25_MAPPED_NATIVE)
  message(FATAL_ERROR "Mapped TYPE25 CUDA qualification requires the independent native packet")
endif()
enable_language(CUDA)
add_subdirectory("${tl_root}/lib_src/solvers" nodal-owner)
include("${tl_root}/lib_src/elements/type25/Type25Batch.cmake")
add_executable(type25_mapped_cuda_test OwnerFixture.cu OwnerTest.cu ../rigid_assembly_owner/Fixture.cpp)
target_link_libraries(type25_mapped_cuda_test PRIVATE tl_type25_batch type25_mapped_native_adapter
  tl_tied_cin_attachment GTest::gtest_main CUDA::cudart)
set_target_properties(type25_mapped_cuda_test PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
target_compile_options(type25_mapped_cuda_test PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME type25_mapped_cuda COMMAND type25_mapped_cuda_test)
set_tests_properties(type25_mapped_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
