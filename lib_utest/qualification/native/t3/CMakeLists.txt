cmake_minimum_required(VERSION 3.22)
project(t3_r1_native_oracle LANGUAGES CXX Fortran)

# Standalone qualification or an explicitly enabled app subdirectory. No CUDA,
# full starter/engine solver, QEPH library link or compiler installation.
option(T3_R1_BUILD_TESTS "Build bounded native T3 startup GTests" ON)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Native T3 R1 currently qualifies GNU Fortran only")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)

set(t3_prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(t3_modules "${CMAKE_CURRENT_BINARY_DIR}/fortran-modules")
set(t3_shared_original "${CMAKE_CURRENT_SOURCE_DIR}/../qeph/original")
get_filename_component(t3_tl_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
file(MAKE_DIRECTORY "${t3_modules}")

execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
  RESULT_VARIABLE t3_verify_status OUTPUT_VARIABLE t3_verify_output ERROR_VARIABLE t3_verify_error)
if(NOT t3_verify_status EQUAL 0)
  message(FATAL_ERROR "Pinned T3 source verification failed: ${t3_verify_error}")
endif()
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${t3_prepared}"
  RESULT_VARIABLE t3_prepare_status OUTPUT_VARIABLE t3_prepare_output ERROR_VARIABLE t3_prepare_error)
if(NOT t3_prepare_status EQUAL 0)
  message(FATAL_ERROR "Private T3 source preparation failed: ${t3_prepare_error}")
endif()

# Reconfigure only when preparation inputs change. Both independent checks also
# run before every actual target build, detecting altered originals or stale
# prepared bytes even when a cached object would otherwise be reused.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/NativeT3Startup.F"
  "${CMAKE_CURRENT_SOURCE_DIR}/extracted/StarterC3evec3.F"
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/source-manifest.json"
  "${t3_shared_original}/common_source/modules/constant_mod.F"
  "${t3_shared_original}/engine/share/spe_inc/implicit_f.inc")
add_custom_target(t3_r1_verify_sources
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py" VERBATIM)
add_custom_target(t3_r1_check_prepared
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${t3_prepared}" --check VERBATIM)

add_library(t3_r1_native STATIC T3Reference.cpp
  "${t3_prepared}/NativeT3Startup.F"
  "${t3_prepared}/extracted/StarterC3evec3.F"
  "${t3_prepared}/original/common_source/modules/constant_mod.F")
add_dependencies(t3_r1_native t3_r1_verify_sources t3_r1_check_prepared)
set_target_properties(t3_r1_native PROPERTIES Fortran_MODULE_DIRECTORY "${t3_modules}")
target_compile_features(t3_r1_native PUBLIC cxx_std_17)
target_compile_options(t3_r1_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-openmp;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
target_compile_definitions(t3_r1_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:MYREAL8;CPP_mach=CPP_p4linux964;COMP_GFORTRAN;C3EVEC3=TL_T3_R1_START_C3EVEC3;c3evec3=TL_T3_R1_START_C3EVEC3>")
target_include_directories(t3_r1_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${t3_tl_root}" PRIVATE
  "${t3_modules}" "${t3_prepared}/original/engine/share/spe_inc"
  "${t3_shared_original}/engine/share/spe_inc"
  "${t3_shared_original}/engine/share/includes"
  "${t3_shared_original}/engine/share/r8"
  "${t3_shared_original}/starter/share/includes")
target_link_libraries(t3_r1_native PRIVATE Threads::Threads)

if(T3_R1_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(t3_startup_check T3StartupTest.cpp)
  target_link_libraries(t3_startup_check PRIVATE t3_r1_native GTest::gtest_main)
  target_compile_options(t3_startup_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME t3_startup_check COMMAND t3_startup_check)
  set_tests_properties(t3_startup_check PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
