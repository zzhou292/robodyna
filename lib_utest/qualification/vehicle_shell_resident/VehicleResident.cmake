include_guard(GLOBAL)
add_executable(vehicle_shell_resident_check
  "${CMAKE_CURRENT_LIST_DIR}/VehicleResidentStartup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VehicleResidentStep.cu"
  "${CMAKE_CURRENT_LIST_DIR}/VehicleResidentChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VehicleResidentTest.cu"
  "${CMAKE_CURRENT_LIST_DIR}/VehicleResidentAdmissionTest.cu")
target_include_directories(vehicle_shell_resident_check PRIVATE "${TL_ROOT}")
target_link_libraries(vehicle_shell_resident_check PRIVATE tl_shell_batch_publication CUDA::cudart GTest::gtest_main)
set_target_properties(vehicle_shell_resident_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(vehicle_shell_resident_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME vehicle_shell_resident_check COMMAND vehicle_shell_resident_check)
set_tests_properties(vehicle_shell_resident_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
