cmake_minimum_required(VERSION 3.22)
project(TLFinalizerLocalControl LANGUAGES CXX)
enable_testing()
find_package(GTest REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(previous "${CMAKE_CURRENT_LIST_DIR}/../solid_candidate_validation")
set(oracle "${CMAKE_CURRENT_BINARY_DIR}/oracle")
option(FINALIZER_LOCAL_CONTROL_CUDA "Actual finalizer/control CUDA comparisons" OFF)
if(FINALIZER_LOCAL_CONTROL_CUDA)
  enable_language(CUDA)
  add_subdirectory("${TL_ROOT}/lib_src/solvers" nodal-owner)
endif()
add_subdirectory("${TL_ROOT}/lib_src/elements/solids/resident" solid-values)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/baseline.json" "${CMAKE_CURRENT_LIST_DIR}/transformations.json"
  "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json" "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py"
  "${CMAKE_CURRENT_LIST_DIR}/generate_oracles.py"
  "${TL_ROOT}/lib_src/elements/solids/resident/Candidate.cu")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py"
  COMMAND_ERROR_IS_FATAL ANY)
foreach(generator IN ITEMS "${previous}/generate_oracle.py" "${CMAKE_CURRENT_LIST_DIR}/generate_oracles.py")
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${generator}" --output "${oracle}"
    COMMAND_ERROR_IS_FATAL ANY)
endforeach()
add_executable(finalizer_local_control_host QbatHostTest.cpp SolidHostTest.cpp
  "${TL_ROOT}/lib_src/elements/qbat/QbatBatchArena.cpp"
  "${TL_ROOT}/lib_src/elements/qbat/QbatBatchIdentity.cpp")
target_include_directories(finalizer_local_control_host PRIVATE "${TL_ROOT}" "${oracle}")
target_link_libraries(finalizer_local_control_host PRIVATE tl_solid_batch_values GTest::gtest_main CUDA::toolkit)
target_compile_features(finalizer_local_control_host PRIVATE cxx_std_17)
target_compile_options(finalizer_local_control_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME finalizer_local_control_host COMMAND finalizer_local_control_host)
add_test(NAME finalizer_local_control_source COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/verify_sources.py")
if(FINALIZER_LOCAL_CONTROL_CUDA)
  include("${TL_ROOT}/lib_src/elements/qbat/QbatBatch.cmake")
  add_executable(qbat_local_control_cuda QbatCudaTest.cu)
  target_link_libraries(qbat_local_control_cuda PRIVATE tl_qbat_batch GTest::gtest_main CUDA::cudart)
  add_executable(solid_local_control_cuda SolidCudaTest.cu SolidCurrentCandidate.cu
    "${previous}/FrozenCandidate.cu"
    "${TL_ROOT}/lib_src/elements/solids/resident/ResultValidation.cu")
  target_link_libraries(solid_local_control_cuda PRIVATE tl_solid_batch_values GTest::gtest_main CUDA::cudart)
  foreach(target IN ITEMS qbat_local_control_cuda solid_local_control_cuda)
    target_include_directories(${target} PRIVATE "${TL_ROOT}" "${oracle}")
    set_target_properties(${target} PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
    target_compile_options(${target} PRIVATE
      $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
    add_test(NAME ${target} COMMAND ${target})
  endforeach()
endif()
get_property(checks DIRECTORY PROPERTY TESTS)
set_tests_properties(${checks} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
