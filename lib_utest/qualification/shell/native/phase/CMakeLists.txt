cmake_minimum_required(VERSION 3.22)

# Opt-in qualification subdirectory; surrounding CMake enables Fortran and
# creates chvis3_native with the exact shared original modules and includes.
if(NOT TARGET chvis3_native)
  message(FATAL_ERROR "shell_phase_native requires the exact native/chvis3 target")
endif()
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Native phase oracle currently qualifies GNU Fortran only")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
  RESULT_VARIABLE phase_verify_status OUTPUT_VARIABLE phase_verify_output ERROR_VARIABLE phase_verify_error)
if(NOT phase_verify_status EQUAL 0)
  message(FATAL_ERROR "Pinned phase source verification failed: ${phase_verify_error}")
endif()
add_custom_target(shell_phase_verify_sources
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
  VERBATIM COMMENT "Verify unchanged native shell phase source closure")
set(phase_original "${CMAKE_CURRENT_SOURCE_DIR}/original")
set(phase_shared_original "${CMAKE_CURRENT_SOURCE_DIR}/../chvis3/original")
get_target_property(phase_shared_modules chvis3_native Fortran_MODULE_DIRECTORY)
if(NOT phase_shared_modules)
  message(FATAL_ERROR "chvis3_native must expose its Fortran_MODULE_DIRECTORY")
endif()
add_library(shell_phase_native SHARED ShellPhaseNative.cpp NativeShellPhase.F
  "${phase_original}/engine/source/elements/shell/coque/ccoor3.F"
  "${phase_original}/engine/source/elements/shell/coque/cderi3.F"
  "${phase_original}/engine/source/elements/shell/coque/ccoef3.F"
  "${phase_original}/engine/source/elements/shell/coque/cdefo3.F"
  "${phase_original}/engine/source/elements/shell/coque/ccurv3.F"
  "${phase_original}/engine/source/elements/shell/coque/cstra3.F")
add_dependencies(shell_phase_native shell_phase_verify_sources chvis3_native)
target_compile_features(shell_phase_native PRIVATE cxx_std_17)
target_compile_options(shell_phase_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
# Unlike CHVIS3, CSTRA3 includes vectorize.inc. Its exact GNU branch requires
# this platform selector. Both selectors use the same MVSIZ=129 and r8 kinds.
target_compile_definitions(shell_phase_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:CPP_mach=CPP_p4linux964;COMP_GFORTRAN>")
target_include_directories(shell_phase_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" PRIVATE
  "${phase_shared_modules}"
  "${phase_original}/engine/share/includes"
  "${phase_original}/engine/share/spe_inc"
  "${phase_shared_original}/engine/share/includes"
  "${phase_shared_original}/engine/share/spe_inc"
  "${phase_shared_original}/engine/share/r8")
target_link_libraries(shell_phase_native PRIVATE chvis3_native Threads::Threads)
add_test(NAME shell_phase_native_analytic
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_native.py"
          --library "$<TARGET_FILE:shell_phase_native>"
          --report "${CMAKE_CURRENT_BINARY_DIR}/phase-native-tests.json")
set_tests_properties(shell_phase_native_analytic PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)

# Separate current-frame oracle: complete unchanged routines and real native
# derived types. This does not alter the supplied-frame phase API or arithmetic.
set(frame_modules "${CMAKE_CURRENT_BINARY_DIR}/frame-fortran-modules")
file(MAKE_DIRECTORY "${frame_modules}")
add_library(shell_frame_native SHARED ShellFrameNative.cpp NativeShellFrame.F
  "${phase_original}/common_source/modules/precision_mod.F90"
  "${phase_original}/common_source/modules/mat_elem/elbufdef_mod.F90"
  "${phase_original}/engine/source/elements/shell/coque/cnvec3.F"
  "${phase_original}/engine/source/elements/shell/coque/cortdir3.F")
set_target_properties(shell_frame_native PROPERTIES Fortran_MODULE_DIRECTORY "${frame_modules}")
add_dependencies(shell_frame_native shell_phase_verify_sources chvis3_native)
target_compile_features(shell_frame_native PRIVATE cxx_std_17)
target_compile_options(shell_frame_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
target_compile_definitions(shell_frame_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:MYREAL8;CPP_mach=CPP_p4linux964;COMP_GFORTRAN>")
target_include_directories(shell_frame_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" PRIVATE
  "${frame_modules}" "${phase_shared_modules}"
  "${phase_original}/engine/share/includes"
  "${phase_original}/engine/share/spe_inc"
  "${phase_shared_original}/engine/share/includes"
  "${phase_shared_original}/engine/share/spe_inc"
  "${phase_shared_original}/engine/share/r8")
target_link_libraries(shell_frame_native PRIVATE chvis3_native Threads::Threads)
add_test(NAME shell_frame_native_analytic
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_native_frame.py"
    --library "$<TARGET_FILE:shell_frame_native>"
    --phase-library "$<TARGET_FILE:shell_phase_native>"
    --report "${CMAKE_CURRENT_BINARY_DIR}/frame-native-tests.json")
set_tests_properties(shell_frame_native_analytic PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
