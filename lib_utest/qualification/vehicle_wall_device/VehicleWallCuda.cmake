include_guard(GLOBAL)
add_executable(vehicle_wall_device_check VehicleWallFixture.cpp VehicleWallChecks.cpp VehicleWallSourceTest.cpp
  VehicleWallCudaTest.cu VehicleWallFailureTest.cu
  "${TL_ROOT}/lib_utest/qualification/surface_contact/NodalWallCapacityCudaProbe.cpp")
target_link_libraries(vehicle_wall_device_check PRIVATE tl_nodal_wall_contact_device tl_shell_batch_binding CUDA::cudart GTest::gtest_main)
target_include_directories(vehicle_wall_device_check PRIVATE "${TL_ROOT}")
target_link_options(vehicle_wall_device_check PRIVATE "LINKER:--wrap=cudaMalloc" "LINKER:--wrap=cudaMemcpyAsync")
set_target_properties(vehicle_wall_device_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(vehicle_wall_device_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME vehicle_wall_device_check COMMAND vehicle_wall_device_check)
set_tests_properties(vehicle_wall_device_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
