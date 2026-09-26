cmake_minimum_required(VERSION 3.22)
project(QbatPrivateTrialQualification LANGUAGES CXX CUDA Fortran)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(QBAT_RESIDENT_NATIVE ON CACHE BOOL "Existing whole native recurrence" FORCE)
set(QBAT_RESIDENT_CUDA ON CACHE BOOL "Existing actual owner publication" FORCE)
add_subdirectory("../qbat_resident" resident)
add_executable(qbat_private_trial_host HostTest.cpp ../qbat_force/NativeOracle.cpp)
target_link_libraries(qbat_private_trial_host PRIVATE tl_qbat_batch_values qbat_force_native GTest::gtest_main)
target_compile_options(qbat_private_trial_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qbat_private_trial_host COMMAND qbat_private_trial_host)

function(qbat_private_cuda target)
  target_link_libraries(${target} PRIVATE tl_qbat_batch_values GTest::gtest_main CUDA::cudart)
  set_target_properties(${target} PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(${target} PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME ${target} COMMAND ${target})
endfunction()
add_executable(qbat_private_trial_cuda CudaTest.cu)
qbat_private_cuda(qbat_private_trial_cuda)
add_executable(qbat_private_public_cuda ../qbat_force/MathCudaTest.cu)
qbat_private_cuda(qbat_private_public_cuda)

# Existing real mapped physical owner/CIN source and partial-trial retry tests.
include("${tl_root}/lib_src/constraints/NodalRigidAssemblyBinding.cmake")
include("${tl_root}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
add_executable(qbat_private_mapped_cuda ../qbat_mapped/OwnerFixture.cu
  ../qbat_mapped/OwnerSourceTest.cu ../qbat_mapped/OwnerTrialTest.cu
  ../rigid_assembly_owner/Fixture.cpp)
qbat_private_cuda(qbat_private_mapped_cuda)
target_link_libraries(qbat_private_mapped_cuda PRIVATE tl_qbat_batch
  tl_nodal_rigid_assembly_binding tl_tied_cin_attachment)

add_test(NAME qbat_private_body_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/verify_force_body.py")
foreach(module IN ITEMS qbat_mapped_gather qbat_measurement_operands)
  add_test(NAME qbat_private_${module}_source COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/../${module}/verify_sources.py")
endforeach()
get_property(tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${tests} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
