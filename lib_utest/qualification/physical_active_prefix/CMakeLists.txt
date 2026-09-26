cmake_minimum_required(VERSION 3.22)
project(TLPhysicalActivePrefix LANGUAGES CXX C CUDA)
enable_testing()
find_package(CUDAToolkit REQUIRED)
set(PHYSICAL_PUBLICATION_CUDA ON CACHE BOOL "Reuse actual physical owner" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../physical_publication" physical-publication)
add_executable(physical_active_prefix_cuda CudaTest.cu
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerStartup.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerAttempt.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerSnapshot.cu")
target_link_libraries(physical_active_prefix_cuda PRIVATE physical_publication_fixture tl_shell_batch_publication CUDA::cudart)
set_target_properties(physical_active_prefix_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(physical_active_prefix_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME physical_active_prefix_cuda COMMAND physical_active_prefix_cuda)
set_tests_properties(physical_active_prefix_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
