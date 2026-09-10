# Included by the existing opt-in contact harness after its nodal owner.
include("${TL_ROOT}/lib_src/collision/NodalWallContactDevice.cmake")
include("${TL_ROOT}/lib_src/elements/ShellBatchBinding.cmake")
add_executable(utest_nodal_wall_model NodalWallModelTest.cpp)
add_executable(utest_nodal_wall_weight_capacity NodalWallWeightCapacityTest.cpp)
add_executable(utest_nodal_wall_owner_cuda NodalWallOwnerTest.cu)
add_executable(utest_nodal_wall_native_cuda
  NodalWallNativeFixture.cpp NodalWallNativeTest.cu)
target_link_libraries(utest_nodal_wall_native_cuda PRIVATE tl_shell_batch_binding)
add_executable(utest_nodal_wall_collection_cuda NodalWallCollectionFixture.cpp NodalWallCollectionTest.cu)
target_link_libraries(utest_nodal_wall_collection_cuda PRIVATE tl_q4_parametric_contact tl_shell_batch_binding)
foreach(check utest_nodal_wall_model utest_nodal_wall_weight_capacity utest_nodal_wall_owner_cuda utest_nodal_wall_native_cuda utest_nodal_wall_collection_cuda)
  target_link_libraries(${check} PRIVATE tl_nodal_wall_contact_device)
  target_compile_options(${check} PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  tl_contact_test(${check})
  set_tests_properties(${check} PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1)
endforeach()
