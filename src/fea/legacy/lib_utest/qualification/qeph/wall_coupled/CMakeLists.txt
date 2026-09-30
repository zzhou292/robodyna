# Explicit short coupled contact qualification; root owns parent opt-in wiring.
if(NOT TARGET qeph_coupled_fixture)
  message(FATAL_ERROR "CW0 requires the existing coupled QEPH qualification fixture")
endif()
include("${TL_ROOT}/lib_src/collision/NodalWallContactDevice.cmake")
add_executable(qeph_wall_coupled_check
  WallCoupledFixture.cu WallCoupledTransaction.cu WallCoupledLedger.cu
  WallCoupledPrefix.cu WallCoupledTest.cu WallCoupledFailureTest.cu)
target_link_libraries(qeph_wall_coupled_check PRIVATE
  qeph_coupled_fixture tl_nodal_wall_contact_device GTest::gtest_main)
set_target_properties(qeph_wall_coupled_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(qeph_wall_coupled_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME qeph_wall_coupled_check COMMAND qeph_wall_coupled_check)
set_tests_properties(qeph_wall_coupled_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
