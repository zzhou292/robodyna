# Configured through the parent focused qualification CMake project.
add_library(shell_fixture STATIC ShellFixture.cu)
# Include the immutable donor CU files privately so checked fixture launchers
# can bypass the original host launchers' process-exit behavior.
target_include_directories(shell_fixture PRIVATE "${SHELL_REFERENCE_CUDA_DIR}")
target_include_directories(shell_fixture PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(shell_fixture PRIVATE MYREAL8)
target_link_libraries(shell_fixture PUBLIC CUDA::cudart)
add_dependencies(shell_fixture verify_shell_sources)

add_executable(shell_spike_tests ShellFixtureTest.cc)
target_link_libraries(shell_spike_tests PRIVATE shell_fixture GTest::gtest_main)
add_test(NAME shell_spike_tests COMMAND shell_spike_tests)

# Preserve distinct private symbols and translation units for both K2 variants.
add_library(elastic_original OBJECT ElasticFixture.cu)
add_library(elastic_gauss OBJECT ElasticFixture.cu)
foreach(target elastic_original elastic_gauss)
  target_include_directories(${target} PRIVATE "${SHELL_REFERENCE_CUDA_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/generated" "${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(${target} PRIVATE MYREAL8)
  add_dependencies(${target} verify_shell_sources)
endforeach()
target_compile_definitions(elastic_gauss PRIVATE SHELL_ELASTIC_GAUSS)
add_library(shell_elastic STATIC ElasticFixture.cc
  $<TARGET_OBJECTS:elastic_original> $<TARGET_OBJECTS:elastic_gauss>)
target_include_directories(shell_elastic PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(shell_elastic PUBLIC CUDA::cudart)
add_executable(shell_elastic_tests ElasticFixtureTest.cc)
target_link_libraries(shell_elastic_tests PRIVATE shell_elastic shell_fixture GTest::gtest_main)
add_test(NAME shell_elastic_tests COMMAND shell_elastic_tests)
set_tests_properties(shell_spike_tests shell_elastic_tests PROPERTIES
  RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
add_test(NAME shell_source_integrity
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/generate_gauss3.py" --self-test --verify)
set_tests_properties(shell_source_integrity PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
