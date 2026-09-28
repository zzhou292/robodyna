cmake_minimum_required(VERSION 3.22)
project(TLCinMassReadTile LANGUAGES CXX CUDA)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(CIN_MASTER_GATHER_CUDA ON CACHE BOOL "Retained complete caller/owner regressions" FORCE)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../cin_master_gather" master-gather)
add_executable(cin_mass_read_tile_cuda CudaTest.cu)
target_include_directories(cin_mass_read_tile_cuda PRIVATE "${tl_root}")
target_link_libraries(cin_mass_read_tile_cuda PRIVATE CUDA::cudart GTest::gtest_main)
set_target_properties(cin_mass_read_tile_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
target_compile_options(cin_mass_read_tile_cuda PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
add_test(NAME cin_mass_read_tile_cuda COMMAND cin_mass_read_tile_cuda)
set_tests_properties(cin_mass_read_tile_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
# Historical source-proof manifests remain frozen. This new proof binds the
# unchanged complete-caller reference and explicitly reviewed production delta.
add_test(NAME cin_mass_read_tile_sources COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
