# The fixture library itself has no Fortran dependency. The independent native
# comparison tests are explicitly enabled by the parent qualification project.
add_library(hourglass_fixture STATIC HourglassFixture.cu)
target_include_directories(hourglass_fixture PRIVATE "${SHELL_REFERENCE_CUDA_DIR}")
target_include_directories(hourglass_fixture PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(hourglass_fixture PRIVATE MYREAL8)
target_link_libraries(hourglass_fixture PUBLIC CUDA::cudart)
add_dependencies(hourglass_fixture verify_shell_sources)
if(TL_SHELL_ENABLE_NATIVE_ORACLES)
  add_executable(shell_hourglass_tests HourglassFixtureTest.cc)
  target_link_libraries(shell_hourglass_tests PRIVATE hourglass_fixture chvis3_native GTest::gtest_main)
  add_test(NAME shell_hourglass_tests COMMAND shell_hourglass_tests)
  set_tests_properties(shell_hourglass_tests PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
