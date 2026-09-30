cmake_minimum_required(VERSION 3.22)
project(RoboNativeGroupedActivity LANGUAGES C CXX CUDA)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/.." adapters)
include("${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/native_group_activity/Tests.cmake")
add_executable(native_group_activity_adapter_cuda AdapterTest.cu
  "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/native_group_activity/Probe.cpp"
  "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_runtime/FullLedgerRig.cu")
target_link_libraries(native_group_activity_adapter_cuda PRIVATE robo_dyna_native_contact_contribution GTest::gtest_main CUDA::cudart)
set_target_properties(native_group_activity_adapter_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(native_group_activity_adapter_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
target_link_options(native_group_activity_adapter_cuda PRIVATE -Wl,--wrap=cudaMemcpyAsync -Wl,--wrap=cudaStreamSynchronize)
add_test(NAME native_group_activity_adapter_cuda COMMAND native_group_activity_adapter_cuda)
add_executable(native_group_direct_cuda "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_multi_interface/CudaTest.cu")
target_include_directories(native_group_direct_cuda PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/adapters/observed-fixture")
target_link_libraries(native_group_direct_cuda PRIVATE tl_radioss_type25_transaction robo_dyna_native_scene_material GTest::gtest_main CUDA::cudart)
set_target_properties(native_group_direct_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(native_group_direct_cuda PRIVATE --fmad=false --prec-div=true --prec-sqrt=true --ftz=false -Xcompiler=-fno-fast-math,-ffp-contract=off)
add_test(NAME native_group_direct_cuda COMMAND native_group_direct_cuda)
set_tests_properties(native_group_activity_adapter_cuda native_group_direct_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)

add_executable(native_group_direct_host "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_multi_interface/HostTest.cpp")
target_link_libraries(native_group_direct_host PRIVATE tl_radioss_type25_transaction GTest::gtest_main)
target_compile_options(native_group_direct_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME native_group_direct_host COMMAND native_group_direct_host)
