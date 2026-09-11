enable_language(CUDA)
enable_language(Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Pinned rigid oracle requires GNU Fortran")
endif()
set(rigid_native "${CMAKE_CURRENT_LIST_DIR}/../nodal_rigid_group/native")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${rigid_native}/verify_sources.py" RESULT_VARIABLE checked)
if(NOT checked EQUAL 0)
  message(FATAL_ERROR "Existing native rigid source identity failed")
endif()
add_library(rigid_assembly_owner_native STATIC
  "${rigid_native}/NativeRigidGroup.F" "${rigid_native}/NativeFrameStep.F"
  "${rigid_native}/NativePrimaryStep.F" "${rigid_native}/NativeMemberStep.F"
  "${rigid_native}/NativeFiniteVelocity.F90" "${rigid_native}/NativeTwoMemberStep.F"
  "${rigid_native}/NativeSchedule.F")
target_include_directories(rigid_assembly_owner_native PRIVATE "${rigid_native}")
target_compile_options(rigid_assembly_owner_native PRIVATE -cpp -ffixed-line-length-none
  -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
add_subdirectory("${TL_ROOT}/lib_src/solvers" "${CMAKE_CURRENT_BINARY_DIR}/owner")
add_executable(rigid_assembly_owner_cuda OwnerTest.cu NativeOwnerTest.cu Fixture.cpp
  ../nodal_rigid_group/GroupStepNativeFixture.cpp ../nodal_rigid_group/GroupPhaseNativeFixture.cpp)
target_link_libraries(rigid_assembly_owner_cuda PRIVATE tl_explicit_nodal_state
  rigid_assembly_owner_native GTest::gtest_main)
set_target_properties(rigid_assembly_owner_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(rigid_assembly_owner_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
add_test(NAME rigid_assembly_owner_cuda COMMAND rigid_assembly_owner_cuda)
add_test(NAME rigid_assembly_owner_native_identity COMMAND "${Python3_EXECUTABLE}" "${rigid_native}/verify_sources.py")
set_tests_properties(rigid_assembly_owner_cuda rigid_assembly_owner_native_identity
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
