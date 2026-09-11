cmake_minimum_required(VERSION 3.22)
project(TLRigidDependentStep LANGUAGES CXX Fortran CUDA)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${TL_ROOT}/lib_src/constraints/NodalRigidGroup.cmake")
set(native "${CMAKE_CURRENT_LIST_DIR}/../nodal_rigid_group/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${native}/verify_sources.py" RESULT_VARIABLE verified)
if(NOT verified EQUAL 0)
  message(FATAL_ERROR "Pinned existing rigid sources failed identity")
endif()
add_library(rigid_dependent_native STATIC "${native}/NativeRigidGroup.F"
  "${native}/NativeFrameStep.F" "${native}/NativePrimaryStep.F" "${native}/NativeMemberStep.F"
  "${native}/NativeFiniteVelocity.F90" "${native}/NativeTwoMemberStep.F" "${native}/NativeSchedule.F")
target_include_directories(rigid_dependent_native PRIVATE "${native}")
target_compile_options(rigid_dependent_native PRIVATE -cpp -ffixed-line-length-none -fcheck=bounds
  -fbacktrace -fno-fast-math -ffp-contract=off)
add_executable(rigid_dependent_check HostTest.cpp NativeTest.cpp
  ../nodal_rigid_group/GroupStepNativeFixture.cpp)
target_link_libraries(rigid_dependent_check PRIVATE tl_nodal_rigid_group rigid_dependent_native GTest::gtest_main)
target_compile_options(rigid_dependent_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME rigid_dependent_values COMMAND rigid_dependent_check)
add_executable(rigid_dependent_cuda_check CudaTest.cu)
target_link_libraries(rigid_dependent_cuda_check PRIVATE tl_nodal_rigid_group GTest::gtest_main)
set_target_properties(rigid_dependent_cuda_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(rigid_dependent_cuda_check PRIVATE --fmad=false --prec-div=true --prec-sqrt=true --ftz=false)
add_test(NAME rigid_dependent_cuda COMMAND rigid_dependent_cuda_check)
add_executable(rigid_dependent_legacy_check ../nodal_rigid_group/GroupStepMathTest.cpp
  ../nodal_rigid_group/GroupStepNativeTest.cpp ../nodal_rigid_group/TwoMemberNativeTest.cpp
  ../nodal_rigid_group/GroupStepNativeFixture.cpp ../nodal_rigid_group/GroupPhaseNativeFixture.cpp)
target_link_libraries(rigid_dependent_legacy_check PRIVATE tl_nodal_rigid_group rigid_dependent_native GTest::gtest_main)
target_compile_options(rigid_dependent_legacy_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME rigid_dependent_legacy COMMAND rigid_dependent_legacy_check)
add_test(NAME rigid_dependent_identity COMMAND "${Python3_EXECUTABLE}" "${native}/verify_sources.py")
set_tests_properties(rigid_dependent_values rigid_dependent_cuda rigid_dependent_legacy rigid_dependent_identity
  PROPERTIES PROCESSORS 1 RUN_SERIAL TRUE TIMEOUT 120)
