cmake_minimum_required(VERSION 3.22)
project(TLSolidCandidateValidation LANGUAGES CXX)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
enable_testing()
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(oracle "${CMAKE_CURRENT_BINARY_DIR}/oracle")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py" COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/generate_oracle.py" --output "${oracle}"
  COMMAND_ERROR_IS_FATAL ANY)
add_subdirectory("${TL_ROOT}/lib_src/elements/solids/resident" resident)
add_executable(solid_candidate_validation_host PredicateTest.cpp MeasureTest.cpp BudgetTest.cpp
  "${oracle}/FrozenArena.cpp")
target_include_directories(solid_candidate_validation_host PRIVATE "${oracle}")
target_link_libraries(solid_candidate_validation_host PRIVATE tl_solid_batch_values GTest::gtest_main)
target_compile_options(solid_candidate_validation_host PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME solid_candidate_validation_host COMMAND solid_candidate_validation_host)
add_test(NAME solid_candidate_validation_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py")
add_test(NAME solid_candidate_validation_resident_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/../extended_solid_resident/verify_sources.py")
add_test(NAME solid_candidate_validation_analytic_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/../solid_law44_analytic/verify_source.py")
set_tests_properties(solid_candidate_validation_host solid_candidate_validation_identity
  solid_candidate_validation_resident_identity solid_candidate_validation_analytic_identity
  PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
option(TL_SOLID_CANDIDATE_VALIDATION_CUDA "Root-only complete frozen candidate comparison" OFF)
if(TL_SOLID_CANDIDATE_VALIDATION_CUDA)
  enable_language(CUDA)
  include("${CMAKE_CURRENT_SOURCE_DIR}/Cuda.cmake")
endif()
