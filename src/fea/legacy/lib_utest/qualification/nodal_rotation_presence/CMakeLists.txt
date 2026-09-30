cmake_minimum_required(VERSION 3.22)
project(TLNodalRotationPresence LANGUAGES CXX CUDA)
enable_testing()
find_package(GTest REQUIRED)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
add_subdirectory("${TL_ROOT}/lib_src/solvers" nodal-owner)
add_executable(nodal_rotation_presence_test PresenceTest.cu ConstraintTest.cu)
target_link_libraries(nodal_rotation_presence_test PRIVATE tl_explicit_nodal_state GTest::gtest_main)
set_target_properties(nodal_rotation_presence_test PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(nodal_rotation_presence_test PRIVATE
  --fmad=false --prec-div=true --prec-sqrt=true --ftz=false -Xcompiler=-fno-fast-math,-ffp-contract=off)
add_test(NAME nodal_rotation_presence COMMAND nodal_rotation_presence_test)
add_executable(nodal_rotation_presence_legacy_test
  ../../utest_nodal_rotation_cuda.cu ../../utest_nodal_temporal_cuda.cu
  ../../utest_nodal_history_admission_cuda.cu
  ../tied_cin_runtime/OwnerTest.cu ../nodal_rigid_group/GroupOwnerStartupTest.cu)
target_link_libraries(nodal_rotation_presence_legacy_test PRIVATE tl_explicit_nodal_state GTest::gtest_main)
set_target_properties(nodal_rotation_presence_legacy_test PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(nodal_rotation_presence_legacy_test PRIVATE
  --fmad=false --prec-div=true --prec-sqrt=true --ftz=false -Xcompiler=-fno-fast-math,-ffp-contract=off)
add_test(NAME nodal_rotation_presence_legacy COMMAND nodal_rotation_presence_legacy_test)
set_tests_properties(nodal_rotation_presence nodal_rotation_presence_legacy
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
