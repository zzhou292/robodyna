# Included by the owning T3 qualification entry point after CUDA setup.
# No alternative project/runner; production remains the existing tl_t3 target.
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/verify_force_sources.py"
  RESULT_VARIABLE t3_force_port_verify_status ERROR_VARIABLE t3_force_port_verify_error)
if(NOT t3_force_port_verify_status EQUAL 0)
  message(FATAL_ERROR "T3 force port verification failed: ${t3_force_port_verify_error}")
endif()
add_custom_target(t3_verify_force_port COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/verify_force_sources.py" VERBATIM)
add_executable(t3_force_port_check T3ForcePortTest.cpp)
add_dependencies(t3_force_port_check t3_verify_force_port)
target_link_libraries(t3_force_port_check PRIVATE tl_t3 t3_r3_native GTest::gtest_main)
target_compile_options(t3_force_port_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME t3_force_port_check COMMAND t3_force_port_check)
set_tests_properties(t3_force_port_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
if(TL_T3_ENABLE_CUDA)
  add_executable(t3_force_port_cuda_check T3ForcePortCudaTest.cu)
  add_dependencies(t3_force_port_cuda_check t3_verify_force_port)
  set_target_properties(t3_force_port_cuda_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_link_libraries(t3_force_port_cuda_check PRIVATE tl_t3 t3_r3_native CUDA::cudart GTest::gtest_main)
  target_compile_options(t3_force_port_cuda_check PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME t3_force_port_cuda_check COMMAND t3_force_port_cuda_check)
  set_tests_properties(t3_force_port_cuda_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
endif()
