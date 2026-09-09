cmake_minimum_required(VERSION 3.22)
project(qeph_q1_native_oracle LANGUAGES CXX Fortran)

# Standalone, explicit opt-in. No CUDA, full solver or installed compiler change.
option(QEPH_Q1_BUILD_TESTS "Build the bounded native Q1 GTests" ON)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "QEPH Q1 currently qualifies GNU Fortran only")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py"
  RESULT_VARIABLE verify_status OUTPUT_VARIABLE verify_output ERROR_VARIABLE verify_error)
if(NOT verify_status EQUAL 0)
  message(FATAL_ERROR "Pinned QEPH Q1 source verification failed: ${verify_error}")
endif()
add_custom_target(qeph_q1_verify_sources
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/verify_sources.py" VERBATIM)
set(original "${CMAKE_CURRENT_SOURCE_DIR}/original")
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/fortran-modules")
file(MAKE_DIRECTORY "${modules}")
get_filename_component(tl_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(native_sources NativeQephStartup.F NativeQephKinematics.F
  extracted/Cneveci.F extracted/StartSkew.F extracted/Cderii.F
  extracted/EngineSkew.F extracted/Czcorc.F extracted/Czcorp5.F
  extracted/Cortdir3.F extracted/A3invdpV.F extracted/Czdef.F
  original/common_source/modules/constant_mod.F
  original/common_source/modules/precision_mod.F90
  original/common_source/modules/elements/element_mod.F90
  original/common_source/modules/mat_elem/elbufdef_mod.F90)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${native_sources} prepare_sources.py verify_sources.py source-manifest.json
  original/engine/share/spe_inc/implicit_f.inc)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE prepare_status ERROR_VARIABLE prepare_error)
if(NOT prepare_status EQUAL 0)
  message(FATAL_ERROR "Q1 module preparation failed: ${prepare_error}")
endif()
add_custom_target(qeph_q1_check_prepared
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check VERBATIM)
list(TRANSFORM native_sources PREPEND "${prepared}/")
add_library(qeph_q1_native STATIC QephReference.cpp QephGeometry.cpp ${native_sources})
add_dependencies(qeph_q1_native qeph_q1_verify_sources)
add_dependencies(qeph_q1_native qeph_q1_check_prepared)
set_target_properties(qeph_q1_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_compile_features(qeph_q1_native PUBLIC cxx_std_17)
target_compile_options(qeph_q1_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
target_compile_definitions(qeph_q1_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:MYREAL8;CPP_mach=CPP_p4linux964;COMP_GFORTRAN>")
# Private native routines, real modules and COMMON blocks prevent accidental
# symbol/context interposition with the earlier native shell qualifications.
foreach(symbol CNEVECI CDERII CZCORC1 CZCORP5 CZCORP5X CORTDIR3 A3INVDP A3INVDP_V CZDEF
               PARAM PARIT SCR05 SCR17 COM08 COM08DP IMPL1 IMPL1_PRIVATE VECT01)
  string(TOLOWER "${symbol}" lower)
  target_compile_definitions(qeph_q1_native PRIVATE
    "$<$<COMPILE_LANGUAGE:Fortran>:${symbol}=QE_Q1_${symbol};${lower}=QE_Q1_${symbol}>")
endforeach()
set_source_files_properties("${prepared}/extracted/Cneveci.F" "${prepared}/extracted/StartSkew.F" PROPERTIES
  COMPILE_DEFINITIONS "CLSKEW3=QE_Q1_START_CLSKEW3")
set_source_files_properties("${prepared}/extracted/Czcorc.F" "${prepared}/extracted/EngineSkew.F" PROPERTIES
  COMPILE_DEFINITIONS "CLSKEW3=QE_Q1_ENGINE_CLSKEW3")
target_include_directories(qeph_q1_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${tl_root}" PRIVATE
  "${modules}" "${prepared}/original/engine/share/spe_inc"
  "${original}/engine/share/spe_inc" "${original}/engine/share/includes"
  "${original}/engine/share/r8" "${original}/starter/share/includes")
target_link_libraries(qeph_q1_native PRIVATE Threads::Threads)

if(QEPH_Q1_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(qeph_kinematics_check QephKinematicsTest.cpp)
  target_link_libraries(qeph_kinematics_check PRIVATE qeph_q1_native GTest::gtest_main)
  target_compile_options(qeph_kinematics_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME qeph_kinematics_check COMMAND qeph_kinematics_check)
  set_tests_properties(qeph_kinematics_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
endif()
