add_executable(mixed_layered_resident_check
  "${CMAKE_CURRENT_LIST_DIR}/MixedResidentStartup.cu"
  "${CMAKE_CURRENT_LIST_DIR}/MixedResidentFrames.cu"
  "${CMAKE_CURRENT_LIST_DIR}/MixedResidentOracle.cu"
  "${CMAKE_CURRENT_LIST_DIR}/MixedResidentTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/MixedReadbackTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/MixedReadFault.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../resident_plasticity/ResidentPlasticityFixture.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../resident_plasticity/ResidentCollectionFixture.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../t3/mixed/MixedShellFixture.cu")
target_include_directories(mixed_layered_resident_check PRIVATE "${tl_root}")
target_link_libraries(mixed_layered_resident_check PRIVATE tl_shell_batch_publication
  qeph_q1_native t3_r3_native CUDA::cudart GTest::gtest_main)
target_link_options(mixed_layered_resident_check PRIVATE "-Wl,--wrap=cudaMemcpyAsync")
set_target_properties(mixed_layered_resident_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(mixed_layered_resident_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME mixed_layered_resident_check COMMAND mixed_layered_resident_check)
set_tests_properties(mixed_layered_resident_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
include("${CMAKE_CURRENT_LIST_DIR}/../resident_plasticity/ResidentPlasticity.cmake")
