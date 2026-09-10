# Explicit short incoming fixture. The root launcher authenticates its final
# wall-screen index before supplying the required selected-step environment.
add_executable(qeph_wall_incoming_check
  WallIncomingFixture.cu WallIncomingChecks.cu WallIncomingPrefix.cu
  WallIncomingFailure.cu WallIncomingTest.cu
  ../wall_coupled/WallCoupledFixture.cu
  ../wall_coupled/WallCoupledTransaction.cu
  ../wall_coupled/WallCoupledLedger.cu
  ../wall_coupled/ScreenedWallFixture.cu
  ../wall_coupled/ScreenedWallChecks.cu)
target_link_libraries(qeph_wall_incoming_check PRIVATE
  qeph_coupled_fixture tl_nodal_wall_contact_device
  qeph_wall_recurrence_analysis GTest::gtest_main)
target_compile_features(qeph_wall_incoming_check PRIVATE cxx_std_17)
set_target_properties(qeph_wall_incoming_check PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(qeph_wall_incoming_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME qeph_wall_incoming_check COMMAND qeph_wall_incoming_check)
set_tests_properties(qeph_wall_incoming_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
