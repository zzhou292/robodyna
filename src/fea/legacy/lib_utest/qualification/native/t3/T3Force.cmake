# Native force/history uses the same R2 modules/COMMON context; never QEPH's.
option(T3_R3_BUILD_TESTS "Build bounded native T3 force/history GTests" ON)
set(t3_force_prepared "${CMAKE_CURRENT_BINARY_DIR}/force-prepared")
set(t3_force_modules "${CMAKE_CURRENT_BINARY_DIR}/force-fortran-modules")
file(MAKE_DIRECTORY "${t3_force_modules}")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --stage force --output "${t3_force_prepared}"
  RESULT_VARIABLE t3_force_prepare_status ERROR_VARIABLE t3_force_prepare_error)
if(NOT t3_force_prepare_status EQUAL 0)
  message(FATAL_ERROR "Private T3 force source preparation failed: ${t3_force_prepare_error}")
endif()
add_custom_target(t3_r3_check_prepared
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --stage force --output "${t3_force_prepared}" --check VERBATIM)
set(t3_force_sources T3NativeHistory.F T3NativeMaterial.F T3NativeLaw1.F T3NativeStiffness.F
  NativeT3Force.F NativeT3Scatter.F extracted/C3Coef3.F extracted/C3Stra3.F extracted/C3Dt3.F
  extracted/C3sroto3.F extracted/C3Fint3.F extracted/C3Fcum3.F extracted/C3Mcum3.F extracted/C3Updt3.F
  extracted/Sigeps01G.F extracted/Cssp2a11.F)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  T3NativeHistory.F T3NativeMaterial.F T3NativeLaw1.F T3NativeStiffness.F
  NativeT3Force.F NativeT3Scatter.F T3Force.cmake)
list(TRANSFORM t3_force_sources PREPEND "${t3_force_prepared}/")
add_library(t3_r3_native STATIC T3History.cpp T3ForceReference.cpp ${t3_force_sources})
add_dependencies(t3_r3_native t3_r1_verify_sources t3_r3_check_prepared)
t3_configure_engine_target(t3_r3_native "${t3_force_prepared}" "${t3_force_modules}")
target_link_libraries(t3_r3_native PUBLIC t3_r2_native PRIVATE Threads::Threads)
if(T3_R3_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(t3_force_check T3ForceTest.cpp)
  target_link_libraries(t3_force_check PRIVATE t3_r3_native GTest::gtest_main)
  target_compile_options(t3_force_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME t3_force_check COMMAND t3_force_check)
  set_tests_properties(t3_force_check PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
