# Included by the existing opt-in contact harness after its nodal owner.
include("${TL_ROOT}/lib_src/collision/NodalWallContactDevice.cmake")
add_executable(utest_nodal_wall_model NodalWallModelTest.cpp)
add_executable(utest_nodal_wall_owner_cuda NodalWallOwnerTest.cu)
foreach(check utest_nodal_wall_model utest_nodal_wall_owner_cuda)
  target_link_libraries(${check} PRIVATE tl_nodal_wall_contact_device)
  target_compile_options(${check} PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  tl_contact_test(${check})
  set_tests_properties(${check} PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1)
endforeach()
