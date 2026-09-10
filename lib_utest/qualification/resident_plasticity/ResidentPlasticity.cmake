# Reuses the existing actual-owner mixed fixture and its already built native
# libraries. New native material equations are qualified by the point suite.
get_filename_component(resident_plasticity_tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
add_executable(resident_plasticity_check
  "${CMAKE_CURRENT_LIST_DIR}/ResidentPlasticityFixture.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentPlasticityTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentRateTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentCollectionFixture.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentCollectionPlasticityTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/ResidentAnalyticPlasticityTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/../t3/mixed/MixedShellFixture.cu")
target_include_directories(resident_plasticity_check PRIVATE "${resident_plasticity_tl_root}")
target_link_libraries(resident_plasticity_check PRIVATE tl_shell_batch_publication
  qeph_q1_native t3_r3_native CUDA::cudart GTest::gtest_main)
set_target_properties(resident_plasticity_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(resident_plasticity_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME resident_plasticity_check COMMAND resident_plasticity_check)
set_tests_properties(resident_plasticity_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)

include("${CMAKE_CURRENT_LIST_DIR}/../plasticity_binding/PlasticityBinding.cmake")
