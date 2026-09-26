cmake_minimum_required(VERSION 3.22)
project(RoboNativeContactContribution LANGUAGES CXX CUDA)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(ROBO_DYNA_TL_ROOT "" CACHE PATH "Explicit qualified TL source checkout")
if(NOT EXISTS "${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25Transaction.h")
  message(FATAL_ERROR "Native contribution qualification requires explicit TL transaction source")
endif()
get_filename_component(app_root "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(TYPE25_SEARCH_CUDA ON)
add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_src/solvers" nodal-owner)
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/qeph/QephBatch.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/t3/T3Batch.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchPublication.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../Contribution.cmake")
include("${app_root}/modelio/native_contact_scene/MaterialBridge.cmake")

# Reuse the original bounded qualified fixture serializer/source pins. No
# Fortran/oracle/executable is linked into the production adapter.
set(runtime_test "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_runtime")
set(observed "${CMAKE_CURRENT_BINARY_DIR}/observed-fixture")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${runtime_test}/prepare_fixture.py"
  --output "${observed}" RESULT_VARIABLE fixture_status)
if(NOT fixture_status EQUAL 0)
  message(FATAL_ERROR "Pinned native scene fixture generation failed")
endif()
file(GLOB fixture_inputs CONFIGURE_DEPENDS "${runtime_test}/fixture/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${fixture_inputs} "${runtime_test}/prepare_fixture.py")

add_executable(native_contribution_host HostTest.cpp)
target_link_libraries(native_contribution_host PRIVATE
  robo_dyna_native_contact_contribution GTest::gtest_main)
add_test(NAME native_contribution_host COMMAND native_contribution_host)

add_executable(native_contribution_cuda CudaTest.cu)
target_include_directories(native_contribution_cuda PRIVATE "${observed}")
target_link_libraries(native_contribution_cuda PRIVATE robo_dyna_native_contact_contribution
  robo_dyna_native_scene_material GTest::gtest_main CUDA::cudart)
set_target_properties(native_contribution_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
add_test(NAME native_contribution_cuda COMMAND native_contribution_cuda)
set_tests_properties(native_contribution_host native_contribution_cuda
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
add_test(NAME native_contribution_observed_sources COMMAND "${Python3_EXECUTABLE}" -B
  "${runtime_test}/prepare_fixture.py" --output "${observed}" --check)
target_compile_options(native_contribution_host PRIVATE -fno-fast-math -ffp-contract=off)
target_compile_options(native_contribution_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")

add_executable(native_group_adapter_host GroupHostTest.cpp)
target_link_libraries(native_group_adapter_host PRIVATE robo_dyna_native_contact_contribution GTest::gtest_main)
add_test(NAME native_group_adapter_host COMMAND native_group_adapter_host)
add_executable(native_group_adapter_cuda GroupCudaTest.cu)
target_include_directories(native_group_adapter_cuda PRIVATE "${observed}")
target_link_libraries(native_group_adapter_cuda PRIVATE robo_dyna_native_contact_contribution
  robo_dyna_native_scene_material GTest::gtest_main CUDA::cudart)
set_target_properties(native_group_adapter_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(native_group_adapter_cuda PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
add_test(NAME native_group_adapter_cuda COMMAND native_group_adapter_cuda)
