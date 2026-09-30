include_guard(GLOBAL)
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/self_contact_transaction/TransactionFixture.h")
  message(FATAL_ERROR "Later-epoch diagnostics coupon requires the maintained TL TransactionFixture.h")
endif()
# Reuse the exact physical owner fixture sources, never an alternate owner or
# manually populated mechanics state. The owning runtime already supplies TL libraries.
set(_failure_owner_fixture "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/physical_publication")
add_executable(robo_dyna_vehicle_failure_later_epoch_check
  "${CMAKE_CURRENT_LIST_DIR}/LaterEpochCaptureTest.cu"
  "${_failure_owner_fixture}/Sources.cpp"
  "${_failure_owner_fixture}/Constraints.cpp"
  "${_failure_owner_fixture}/OwnerStartup.cu"
  "${_failure_owner_fixture}/OwnerAttempt.cu"
  "${_failure_owner_fixture}/OwnerSnapshot.cu")
target_include_directories(robo_dyna_vehicle_failure_later_epoch_check PRIVATE "${ROBO_DYNA_TL_ROOT}")
target_link_libraries(robo_dyna_vehicle_failure_later_epoch_check PRIVATE
  robo_dyna_vehicle_failure_diagnostics robo_dyna_candidate_failure_fixture
  tl_self_contact_transaction tl_solid_batch_values tl_shell_physical_owner_values
  tl_nodal_rigid_assembly_binding tl_tied_cin_attachment tl_shell_batch_publication
  CUDA::cudart GTest::gtest_main)
set_target_properties(robo_dyna_vehicle_failure_later_epoch_check PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo_dyna_vehicle_failure_later_epoch_check PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME vehicle_run_failure_later_epoch_cuda COMMAND robo_dyna_vehicle_failure_later_epoch_check)
set_tests_properties(vehicle_run_failure_later_epoch_cuda PROPERTIES TIMEOUT 180 RUN_SERIAL TRUE
  PROCESSORS 1 RESOURCE_LOCK vehicle_self_contact_gpu LABELS "coupon;GPU;failure-diagnostics;later-epoch")
unset(_failure_owner_fixture)
