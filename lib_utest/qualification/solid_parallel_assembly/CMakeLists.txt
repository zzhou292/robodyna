cmake_minimum_required(VERSION 3.22)
project(TLSolidParallelAssembly LANGUAGES CXX CUDA Fortran C)
find_package(GTest REQUIRED)
enable_testing()
set(TL_EXTENDED_SOLID_RESIDENT_CUDA ON CACHE BOOL "Reuse complete existing owning CUDA gates" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../extended_solid_resident" owning)
target_sources(extended_solid_resident_host PRIVATE "${CMAKE_CURRENT_LIST_DIR}/HostTest.cpp")
target_sources(extended_solid_resident_cuda PRIVATE "${CMAKE_CURRENT_LIST_DIR}/CudaFixture.cu" "${CMAKE_CURRENT_LIST_DIR}/CudaTest.cu")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME solid_parallel_assembly_source COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
target_sources(extended_solid_resident_cuda PRIVATE "${CMAKE_CURRENT_LIST_DIR}/LaunchFault.cpp")
target_link_options(extended_solid_resident_cuda PRIVATE -Wl,--wrap=cudaPeekAtLastError)
add_executable(solid_parallel_assembly_benchmark Benchmark.cu CudaFixture.cu
  ../extended_solid_resident/CudaFixture.cu ../extended_solid_resident/OwnerFixture.cpp
  ../rigid_assembly_owner/Fixture.cpp)
target_link_libraries(solid_parallel_assembly_benchmark PRIVATE tl_solid_batch
  extended_resident_native_checks tl_tied_cin_attachment tl_nodal_rigid_assembly_binding GTest::gtest_main)
set_target_properties(solid_parallel_assembly_benchmark PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(solid_parallel_assembly_benchmark PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
