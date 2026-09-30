enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)
add_subdirectory("${tl_root}/lib_src/solvers" nodal-owner)
include("${tl_root}/lib_src/elements/qbat/QbatBatch.cmake")
add_executable(qbat_mapped_cuda_test OwnerFixture.cu OwnerSourceTest.cu OwnerTrialTest.cu
  ../rigid_assembly_owner/Fixture.cpp)
target_link_libraries(qbat_mapped_cuda_test PRIVATE tl_qbat_batch
  tl_nodal_rigid_assembly_binding tl_tied_cin_attachment GTest::gtest_main CUDA::cudart)
set_target_properties(qbat_mapped_cuda_test PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
target_compile_options(qbat_mapped_cuda_test PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME qbat_mapped_cuda COMMAND qbat_mapped_cuda_test)
set_tests_properties(qbat_mapped_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
