# Reuse the qualified complete small physical fixture, no vehicle parser/model stand-in.
set(wall_physical_fixture "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/physical_publication")
add_library(robo_dyna_loaded_wall_fixture STATIC
  "${wall_physical_fixture}/Sources.cpp" "${wall_physical_fixture}/Constraints.cpp")
target_include_directories(robo_dyna_loaded_wall_fixture PUBLIC "${ROBO_DYNA_TL_ROOT}")
target_link_libraries(robo_dyna_loaded_wall_fixture PUBLIC tl_solid_batch_values
  tl_shell_physical_owner_values tl_nodal_rigid_assembly_binding tl_tied_cin_attachment GTest::gtest_main)
target_compile_features(robo_dyna_loaded_wall_fixture PUBLIC cxx_std_17)
target_compile_options(robo_dyna_loaded_wall_fixture PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_loaded_wall_cuda_check "${CMAKE_CURRENT_LIST_DIR}/LoadedCudaTest.cu"
  "${wall_physical_fixture}/OwnerStartup.cu" "${wall_physical_fixture}/OwnerAttempt.cu"
  "${wall_physical_fixture}/OwnerSnapshot.cu")
target_link_libraries(robo_dyna_loaded_wall_cuda_check PRIVATE robo_dyna_loaded_wall_fixture
  robo_dyna_vehicle_loaded_wall CUDA::cudart)
set_target_properties(robo_dyna_loaded_wall_cuda_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo_dyna_loaded_wall_cuda_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME vehicle_loaded_wall_cuda COMMAND robo_dyna_loaded_wall_cuda_check)
set_tests_properties(vehicle_loaded_wall_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 240)
