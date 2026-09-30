cmake_minimum_required(VERSION 3.22)
project(TLPhysicalActivitySnapshot LANGUAGES CXX C CUDA)
enable_testing()
find_package(CUDAToolkit REQUIRED)
set(PHYSICAL_PUBLICATION_CUDA ON CACHE BOOL "Reuse actual physical owner" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../physical_publication" physical-publication)
add_executable(physical_activity_snapshot_host HostTest.cpp)
target_link_libraries(physical_activity_snapshot_host PRIVATE physical_publication_fixture tl_shell_batch_publication)
add_test(NAME physical_activity_snapshot_host COMMAND physical_activity_snapshot_host)
add_executable(physical_activity_snapshot_cuda AuthTest.cu RemovalTest.cu FaultTest.cu Type45Test.cu
  "${CMAKE_CURRENT_LIST_DIR}/../type45_resident/OwnerFixture.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../type45_resident/OwnerScatter.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerStartup.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerAttempt.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerSnapshot.cu")
target_link_libraries(physical_activity_snapshot_cuda PRIVATE physical_publication_fixture tl_shell_batch_publication CUDA::cudart)
target_link_options(physical_activity_snapshot_cuda PRIVATE "LINKER:--wrap=cudaMemcpyAsync")
set_target_properties(physical_activity_snapshot_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(physical_activity_snapshot_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME physical_activity_snapshot_cuda COMMAND physical_activity_snapshot_cuda)
set_tests_properties(physical_activity_snapshot_host physical_activity_snapshot_cuda
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)

# Unchanged owning legacy guard: real material removal must still reject.
add_executable(physical_activity_legacy_prefix_cuda
  "${CMAKE_CURRENT_LIST_DIR}/../physical_active_prefix/CudaTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerStartup.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerAttempt.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../physical_publication/OwnerSnapshot.cu")
target_link_libraries(physical_activity_legacy_prefix_cuda PRIVATE physical_publication_fixture tl_shell_batch_publication CUDA::cudart)
set_target_properties(physical_activity_legacy_prefix_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(physical_activity_legacy_prefix_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME physical_activity_legacy_prefix_cuda COMMAND physical_activity_legacy_prefix_cuda)
set_tests_properties(physical_activity_legacy_prefix_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
