cmake_minimum_required(VERSION 3.22)
project(chvis3_native_oracle LANGUAGES CXX Fortran)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Native oracle currently qualifies the isolated GNU Fortran context only")
endif()
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
                RESULT_VARIABLE verify_status OUTPUT_VARIABLE verify_output ERROR_VARIABLE verify_error)
if(NOT verify_status EQUAL 0)
  message(FATAL_ERROR "Pinned native source verification failed: ${verify_error}")
endif()
add_custom_target(chvis3_verify_sources
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
  VERBATIM COMMENT "Verify unchanged native CHVIS3 dependency closure")

set(native_original "${CMAKE_CURRENT_SOURCE_DIR}/original")
set(native_module_dir "${CMAKE_CURRENT_BINARY_DIR}/fortran-modules")
file(MAKE_DIRECTORY "${native_module_dir}")
add_library(chvis3_native SHARED
  HourglassNative.cpp NativeHourglass.F
  "${native_original}/common_source/modules/constant_mod.F"
  "${native_original}/common_source/modules/elements/element_mod.F90"
  "${native_original}/engine/source/elements/shell/coque/chvis3.F")
add_dependencies(chvis3_native chvis3_verify_sources)
set_target_properties(chvis3_native PROPERTIES Fortran_MODULE_DIRECTORY "${native_module_dir}")
target_compile_features(chvis3_native PRIVATE cxx_std_17)
target_compile_options(chvis3_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>")
target_compile_definitions(chvis3_native PRIVATE "$<$<COMPILE_LANGUAGE:Fortran>:CPP_mach=CPP_linux964>")
target_include_directories(chvis3_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" PRIVATE
  "${native_module_dir}"
  "${native_original}/engine/share/spe_inc"
  "${native_original}/engine/share/includes"
  "${native_original}/engine/share/r8")
target_link_libraries(chvis3_native PRIVATE Threads::Threads)

enable_testing()
add_test(NAME chvis3_native_analytic
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_native.py"
          --library "$<TARGET_FILE:chvis3_native>"
          --report "${CMAKE_CURRENT_BINARY_DIR}/native-tests.json")
set_tests_properties(chvis3_native_analytic PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
