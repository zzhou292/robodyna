cmake_minimum_required(VERSION 3.22)
project(law44_point_native_reference LANGUAGES CXX Fortran)

option(LAW44_POINT_BUILD_TESTS "Build focused tabulated plane-stress native checks" ON)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "LAW44 point reference currently qualifies GNU Fortran only")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(prepared "${CMAKE_CURRENT_BINARY_DIR}/prepared")
set(modules "${CMAKE_CURRENT_BINARY_DIR}/fortran-modules")
set(shared "${CMAKE_CURRENT_SOURCE_DIR}/../qeph/original")
get_filename_component(tl_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --output "${prepared}" RESULT_VARIABLE status ERROR_VARIABLE diagnostic)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "LAW44 point source preparation failed: ${diagnostic}")
endif()
file(MAKE_DIRECTORY "${modules}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  prepare_sources.py source-manifest.json original/sigeps44c.F original/vinter.F
  original/coqini.F original/com20_c.inc)
add_custom_target(law44_point_verify_sources
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --output "${prepared}" --check VERBATIM)
add_library(law44_point_native STATIC NativePoint.cpp NativePoint.F NativeSection.F
  "${prepared}/Sigeps44c.F" "${prepared}/Vinter.F"
  "${prepared}/Coqini.F"
  "${prepared}/Constant.F" "${prepared}/Precision.F90")
add_dependencies(law44_point_native law44_point_verify_sources)
set_target_properties(law44_point_native PROPERTIES Fortran_MODULE_DIRECTORY "${modules}")
target_compile_features(law44_point_native PUBLIC cxx_std_17)
target_compile_options(law44_point_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
target_compile_definitions(law44_point_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:MYREAL8;CPP_mach=CPP_p4linux964;COMP_GFORTRAN>")
target_include_directories(law44_point_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${tl_root}"
  PRIVATE "${prepared}" "${modules}" "${shared}/engine/share/spe_inc"
  "${shared}/engine/share/includes" "${shared}/engine/share/r8")
if(LAW44_POINT_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(law44_point_native_check NativePointTest.cpp)
  target_link_libraries(law44_point_native_check PRIVATE law44_point_native GTest::gtest_main)
  target_compile_options(law44_point_native_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME law44_point_native COMMAND law44_point_native_check)
  set_tests_properties(law44_point_native PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
