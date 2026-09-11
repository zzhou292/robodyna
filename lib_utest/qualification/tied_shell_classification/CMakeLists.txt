cmake_minimum_required(VERSION 3.22)
project(TLTiedShellClassification LANGUAGES CXX)
enable_testing()
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${tl_root}/lib_src/constraints/tied_shell/TiedClassification.cmake")
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_executable(tied_classification_host_test ValueTest.cpp FailureTest.cpp)
target_link_libraries(tied_classification_host_test PRIVATE tl_tied_shell_classification GTest::gtest_main)
add_test(NAME tied_classification_values COMMAND tied_classification_host_test)
set(native "${CMAKE_CURRENT_LIST_DIR}/native")
add_test(NAME tied_classification_source_identity COMMAND "${Python3_EXECUTABLE}" -B "${native}/verify_sources.py")
option(TL_TIED_CLASSIFICATION_NATIVE "Build complete original native classifier checks" OFF)
if(TL_TIED_CLASSIFICATION_NATIVE)
  enable_language(Fortran)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/modules")
  add_library(tied_classification_native STATIC native/Context.F90
    native/NativeInterfaces.F native/Packet.F native/RigidPacket.F native/Intab.F
    native/original/starter/source/interfaces/inter3d1/itagsl2.F
    native/original/starter/source/constraints/general/kinset.F
    native/original/starter/source/constraints/general/kinini.F)
  target_include_directories(tied_classification_native PRIVATE "${native}/context" "${native}"
    "${native}/original/common_source/includes")
  target_compile_options(tied_classification_native PRIVATE -cpp -ffixed-line-length-none
    -ffree-line-length-none -fcheck=all -fno-fast-math -ffp-contract=off)
  set_target_properties(tied_classification_native PROPERTIES
    Fortran_MODULE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/modules")
  add_executable(tied_classification_native_test NativeOracle.cpp NativeTest.cpp)
  target_link_libraries(tied_classification_native_test PRIVATE
    tl_tied_shell_classification tied_classification_native GTest::gtest_main)
  add_test(NAME tied_classification_native COMMAND tied_classification_native_test)
endif()
get_property(tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${tests} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
