cmake_minimum_required(VERSION 3.22)
project(TLExtendedSolidResident LANGUAGES CXX)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
enable_testing()
find_package(GTest REQUIRED)
option(TL_EXTENDED_SOLID_RESIDENT_CUDA "Root-only actual five-family owner/native gate" OFF)
if(TL_EXTENDED_SOLID_RESIDENT_CUDA)
  enable_language(CUDA)
  enable_language(Fortran)
  add_subdirectory("${TL_ROOT}/lib_src/solvers" owner)
  set(TL_SOLID_BATCH_CUDA ON)
endif()
add_subdirectory("${TL_ROOT}/lib_src/elements/solids/resident" resident)
include("${TL_ROOT}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
include("${TL_ROOT}/lib_src/constraints/NodalRigidAssemblyBinding.cmake")
add_executable(extended_solid_resident_host RelocateTest.cpp StateTest.cpp BudgetTest.cpp
  OwnerFixtureTest.cpp OwnerFixture.cpp ../rigid_assembly_owner/Fixture.cpp)
target_link_libraries(extended_solid_resident_host PRIVATE tl_solid_batch_values tl_tied_cin_attachment tl_nodal_rigid_assembly_binding GTest::gtest_main)
target_compile_options(extended_solid_resident_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME extended_solid_resident_host COMMAND extended_solid_resident_host)
set_tests_properties(extended_solid_resident_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)

if(TL_EXTENDED_SOLID_RESIDENT_CUDA)
  include("${CMAKE_CURRENT_LIST_DIR}/Native.cmake")
  add_executable(extended_solid_resident_cuda OwnerTest.cu CudaFixture.cu OwnerFixture.cpp
    ../rigid_assembly_owner/Fixture.cpp)
  target_link_libraries(extended_solid_resident_cuda PRIVATE tl_solid_batch
    extended_resident_native_checks tl_tied_cin_attachment tl_nodal_rigid_assembly_binding GTest::gtest_main)
  set_target_properties(extended_solid_resident_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(extended_solid_resident_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
  add_test(NAME extended_solid_resident_cuda COMMAND extended_solid_resident_cuda)
  set_tests_properties(extended_solid_resident_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
endif()

option(TL_EXTENDED_SOLID_RESIDENT_ORIGINAL "Root-only complete4063 constructor and native source gate" OFF)
set(TL_LAW90_RADIATOR_FIXTURE "" CACHE PATH "Authenticated original radiator geometry")
if(TL_EXTENDED_SOLID_RESIDENT_ORIGINAL)
  if(NOT TL_EXTENDED_SOLID_RESIDENT_CUDA OR NOT REAR18_SOURCE_FIXTURE OR NOT TL_LAW90_RADIATOR_FIXTURE)
    message(FATAL_ERROR "Original resident gate requires CUDA/native and both authenticated fixture paths")
  endif()
  set(fixture "${CMAKE_CURRENT_BINARY_DIR}/source_fixture")
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/../solid18_law44_reference/prepare_fixture.py"
    --fixture "${REAR18_SOURCE_FIXTURE}" --output "${fixture}/RearOriginalFixture.h" RESULT_VARIABLE rear_status)
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/../law90_solid18_reference/source_fixture/prepare_fixture.py"
    --input "${TL_LAW90_RADIATOR_FIXTURE}" --output "${fixture}/RadiatorSourceFixture.h" RESULT_VARIABLE foam_status)
  if(NOT rear_status EQUAL 0 OR NOT foam_status EQUAL 0)
    message(FATAL_ERROR "Original fixture authentication failed")
  endif()
  target_sources(extended_solid_resident_cuda PRIVATE OriginalTest.cpp)
  target_include_directories(extended_solid_resident_cuda PRIVATE "${fixture}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../solid18_law44_reference")
endif()

add_executable(extended_solid_resident_legacy ../solid_resident/HostTest.cpp ../solid_resident/Scratch18Test.cpp)
target_link_libraries(extended_solid_resident_legacy PRIVATE tl_solid_batch_values GTest::gtest_main)
target_compile_options(extended_solid_resident_legacy PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME extended_solid_resident_legacy COMMAND extended_solid_resident_legacy)
set_tests_properties(extended_solid_resident_legacy PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
option(TL_EXTENDED_SOLID_RESIDENT_IDENTITY_CHECK "Check the frozen extended-profile source manifest" ON)
if(TL_EXTENDED_SOLID_RESIDENT_IDENTITY_CHECK)
  add_test(NAME extended_solid_resident_identity COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py")
  set_tests_properties(extended_solid_resident_identity PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
endif()
