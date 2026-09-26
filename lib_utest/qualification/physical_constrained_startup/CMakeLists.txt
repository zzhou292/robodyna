cmake_minimum_required(VERSION 3.22)
project(TLConstrainedPhysicalStartup LANGUAGES C CXX CUDA)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(BEAM18_PUBLICATION_CUDA ON)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../beam18_publication" beam-publication-regression)
add_executable(constrained_startup_host HostTest.cpp)
target_link_libraries(constrained_startup_host PRIVATE tl_shell_batch_publication GTest::gtest_main)
add_test(NAME constrained_startup_host COMMAND constrained_startup_host)
add_executable(constrained_startup_cuda CudaTest.cu FamilyTest.cu ExtendedTest.cu
  ../physical_publication/OwnerStartup.cu ../physical_publication/OwnerAttempt.cu
  ../physical_publication/OwnerSnapshot.cu
  ../beam18_publication/Startup.cu ../beam18_publication/Attempt.cu ../beam18_publication/Values.cpp
  ../type45_resident/OwnerFixture.cu ../type45_resident/OwnerScatter.cu
  ../extended_solid_resident/OwnerFixture.cpp ../rigid_assembly_owner/Fixture.cpp)
target_link_libraries(constrained_startup_cuda PRIVATE beam18_publication_source
  tl_shell_batch_publication CUDA::cudart GTest::gtest_main)
set_target_properties(constrained_startup_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(constrained_startup_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME constrained_startup_cuda COMMAND constrained_startup_cuda)
set_tests_properties(constrained_startup_host constrained_startup_cuda PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
add_test(NAME constrained_startup_extended_sources COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../extended_solid_resident/verify_sources.py")
add_test(NAME constrained_startup_analytic_sources COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/../solid_law44_analytic/verify_source.py")
