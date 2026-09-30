enable_language(Fortran)
# Failure creates the single native LAW44 target. The existing recurrence owner
# then reuses it and supplies complete QEPH/T3 geometry, coefficients and forces.
set(TL_LAYERED_FAILURE_NATIVE ON CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../shell_layered_failure" failure-section)
set(TL_SHELL_LAYERED_J2_ENABLE_CUDA OFF CACHE BOOL "" FORCE)
set(TL_SHELL_LAYERED_NATIVE_RECURRENCE ON CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../shell_layered_j2" layered-reference)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(ff_source "${CMAKE_CURRENT_SOURCE_DIR}/native")
set(ff_prepared "${CMAKE_CURRENT_BINARY_DIR}/failure-force-prepared")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${ff_source}/prepare_sources.py"
  --output "${ff_prepared}" RESULT_VARIABLE ff_status ERROR_VARIABLE ff_diagnostic)
if(NOT ff_status EQUAL 0)
  message(FATAL_ERROR "Failure force native source preparation failed: ${ff_diagnostic}")
endif()
add_custom_target(shell_failure_force_sources
  COMMAND "${Python3_EXECUTABLE}" -B "${ff_source}/prepare_sources.py"
    --output "${ff_prepared}" --check
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/../shell_hardening_continuation/verify_curves.py" VERBATIM)
get_target_property(ff_section_modules shell_layered_failure_native Fortran_MODULE_DIRECTORY)
get_target_property(ff_layered_modules layered_native_qeph Fortran_MODULE_DIRECTORY)
function(ff_family name native)
  add_library(${name} STATIC ${ARGN})
  foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(value ${native} ${property})
    set_property(TARGET ${name} PROPERTY ${property} "${value}")
  endforeach()
  get_target_property(native_modules ${native} Fortran_MODULE_DIRECTORY)
  set(modules "${CMAKE_CURRENT_BINARY_DIR}/${name}-modules")
  file(MAKE_DIRECTORY "${modules}")
  set_target_properties(${name} PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
  target_include_directories(${name} PRIVATE "${native_modules}" "${ff_section_modules}" "${ff_layered_modules}" "${modules}")
  target_link_libraries(${name} PUBLIC ${native} shell_layered_failure_native shell_layered_native_reference)
  add_dependencies(${name} shell_failure_force_sources)
endfunction()
ff_family(shell_failure_qeph_native qeph_q1_native
  "${ff_prepared}/FailureQephSection.F" "${ff_prepared}/FailureQephForce.F")
ff_family(shell_failure_t3_native t3_r3_native
  "${ff_prepared}/FailureT3Section.F" "${ff_prepared}/FailureT3Force.F")
add_executable(shell_failure_force_native_check FailureForceNativeTest.cpp FailureSoundSpeedNativeTest.cpp native/NativePacket.cpp
  "${CMAKE_CURRENT_SOURCE_DIR}/../shell_layered_failure/FailureNativeChecks.cpp")
target_include_directories(shell_failure_force_native_check PRIVATE "${TL_ROOT}")
target_compile_features(shell_failure_force_native_check PRIVATE cxx_std_17)
target_compile_options(shell_failure_force_native_check PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(shell_failure_force_native_check PRIVATE shell_failure_qeph_native shell_failure_t3_native GTest::gtest_main)
add_test(NAME shell_failure_force_native COMMAND shell_failure_force_native_check)
set_tests_properties(shell_failure_force_native PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
if(TL_FAILURE_FORCE_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_executable(shell_failure_force_cuda_check FailureForceCudaTest.cu native/NativePacket.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../shell_layered_failure/FailureNativeChecks.cpp")
  target_include_directories(shell_failure_force_cuda_check PRIVATE "${TL_ROOT}")
  target_link_libraries(shell_failure_force_cuda_check PRIVATE shell_failure_qeph_native shell_failure_t3_native GTest::gtest_main CUDA::cudart)
  target_compile_features(shell_failure_force_cuda_check PRIVATE cxx_std_17 cuda_std_17)
  target_compile_options(shell_failure_force_cuda_check PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME shell_failure_force_cuda COMMAND shell_failure_force_cuda_check)
  set_tests_properties(shell_failure_force_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
