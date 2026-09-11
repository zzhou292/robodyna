cmake_minimum_required(VERSION 3.22)
project(TLShellParentActivity LANGUAGES CXX CUDA)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
add_subdirectory("${tl_root}/lib_src/solvers" nodal-owner)
add_subdirectory("${tl_root}/lib_src/elements/qeph" qeph-values)
add_subdirectory("${tl_root}/lib_src/elements/t3" t3-values)
include("${tl_root}/lib_src/elements/qeph/QephBatch.cmake")
include("${tl_root}/lib_src/elements/t3/T3Batch.cmake")
include("${tl_root}/lib_src/elements/ShellBatchPublication.cmake")
add_executable(shell_parent_activity_cuda_test ActivityTest.cu PreparedActivityTest.cu
  ../qbat_resident/ResidentStartup.cu ../qbat_resident/ResidentFrames.cu
  ../qbat_resident/ReadFault.cu)
target_link_libraries(shell_parent_activity_cuda_test PRIVATE
  tl_shell_batch_publication CUDA::cudart GTest::gtest_main)
target_link_options(shell_parent_activity_cuda_test PRIVATE "-Wl,--wrap=cudaMemcpyAsync")
set_target_properties(shell_parent_activity_cuda_test PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(shell_parent_activity_cuda_test PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME shell_parent_activity_cuda COMMAND shell_parent_activity_cuda_test)
set_tests_properties(shell_parent_activity_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
