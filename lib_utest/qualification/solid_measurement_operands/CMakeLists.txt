cmake_minimum_required(VERSION 3.22)
project(TLSolidMeasurementOperands LANGUAGES CXX)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(previous "${CMAKE_CURRENT_SOURCE_DIR}/../solid_candidate_validation")
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(oracle "${CMAKE_CURRENT_BINARY_DIR}/oracle")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/source-manifest.json"
  "${CMAKE_CURRENT_SOURCE_DIR}/baseline.json"
  "${CMAKE_CURRENT_SOURCE_DIR}/integration-baseline.json"
  "${CMAKE_CURRENT_SOURCE_DIR}/integration_proof.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/operand_proof.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/generate_oracle.py"
  "${previous}/generate_oracle.py")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
  COMMAND_ERROR_IS_FATAL ANY)
foreach(generator IN ITEMS "${previous}/generate_oracle.py" "${CMAKE_CURRENT_SOURCE_DIR}/generate_oracle.py")
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${generator}" --output "${oracle}"
    COMMAND_ERROR_IS_FATAL ANY)
endforeach()
add_subdirectory("${TL_ROOT}/lib_src/elements/solids/resident" resident)
add_executable(solid_measurement_operands_host HostTest.cpp BudgetTest.cpp "${oracle}/OperandBaselineArena.cpp")
target_include_directories(solid_measurement_operands_host PRIVATE "${TL_ROOT}" "${oracle}")
target_link_libraries(solid_measurement_operands_host PRIVATE tl_solid_batch_values GTest::gtest_main)
target_compile_features(solid_measurement_operands_host PRIVATE cxx_std_17)
target_compile_options(solid_measurement_operands_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME solid_measurement_operands_host COMMAND solid_measurement_operands_host)
add_test(NAME solid_measurement_operands_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py")
set_tests_properties(solid_measurement_operands_host solid_measurement_operands_identity
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
option(TL_SOLID_MEASUREMENT_OPERANDS_CUDA "Root-only complete frozen CUDA comparisons" OFF)
if(TL_SOLID_MEASUREMENT_OPERANDS_CUDA)
  enable_language(CUDA)
  add_executable(solid_measurement_operands_cuda CudaTest.cu CurrentCandidate.cu
    "${previous}/CandidateTest.cu" "${previous}/FrozenCandidate.cu"
    "${TL_ROOT}/lib_src/elements/solids/resident/ResultValidation.cu")
  target_include_directories(solid_measurement_operands_cuda PRIVATE "${TL_ROOT}" "${oracle}")
  target_link_libraries(solid_measurement_operands_cuda PRIVATE tl_solid_batch_values GTest::gtest_main)
  set_target_properties(solid_measurement_operands_cuda PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(solid_measurement_operands_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
  add_test(NAME solid_measurement_operands_cuda COMMAND solid_measurement_operands_cuda)
  set_tests_properties(solid_measurement_operands_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 300)
endif()
