cmake_minimum_required(VERSION 3.22)
project(tl_qbat_geometry_qualification LANGUAGES CXX)
enable_testing()
get_filename_component(tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${tl_root}/lib_src/elements/qbat/Qbat.cmake")
find_package(GTest REQUIRED)
add_executable(qbat_geometry_host_test MathTest.cpp)
target_link_libraries(qbat_geometry_host_test PRIVATE tl_qbat_geometry GTest::gtest_main)
target_compile_options(qbat_geometry_host_test PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qbat_geometry_host COMMAND qbat_geometry_host_test)

option(QBAT_NATIVE_CHECKS "Compile selected pinned QBAT geometry reference" ON)
if(QBAT_NATIVE_CHECKS)
  enable_language(Fortran)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(qeph_native "${CMAKE_CURRENT_LIST_DIR}/../native/qeph")
  set(qeph_build "${CMAKE_CURRENT_BINARY_DIR}/native-qeph")
  set(QEPH_Q1_BUILD_TESTS ON CACHE BOOL "Qualify affected native quadrilateral leaves" FORCE)
  add_subdirectory("${qeph_native}" "${qeph_build}")
  add_test(NAME qbat_source_identity COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/native/verify_sources.py")
  add_library(qbat_native STATIC native/NativeCoefficients.F native/NativeGeometry.F
    native/extracted/NativeCbaDef1.F native/extracted/NativeCbaDefSh.F)
  add_dependencies(qbat_native qeph_q1_native)
  target_link_libraries(qbat_native PUBLIC qeph_q1_native)
  target_include_directories(qbat_native PRIVATE "${CMAKE_CURRENT_LIST_DIR}/native"
    "${qeph_build}/fortran-modules" "${qeph_build}/prepared/original/engine/share/spe_inc"
    "${qeph_native}/original/engine/share/spe_inc" "${qeph_native}/original/engine/share/includes"
    "${qeph_native}/original/engine/share/r8" "${qeph_native}/original/starter/share/includes")
  target_compile_definitions(qbat_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN
    CBADEF1=QBAT_CBADEF1 CBADEFSH=QBAT_CBADEFSH)
  target_compile_options(qbat_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
    -fno-fast-math -ffp-contract=off -fcheck=all -finit-real=snan)
  add_executable(qbat_geometry_native_test NativeTest.cpp NativeOracle.cpp)
  target_link_libraries(qbat_geometry_native_test PRIVATE tl_qbat_geometry qbat_native GTest::gtest_main)
  target_compile_options(qbat_geometry_native_test PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME qbat_geometry_native COMMAND qbat_geometry_native_test)
  add_executable(qbat_original_geometry_test OriginalSourceTest.cpp NativeOracle.cpp)
  target_link_libraries(qbat_original_geometry_test PRIVATE tl_qbat_geometry qbat_native GTest::gtest_main)
  target_compile_options(qbat_original_geometry_test PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME qbat_original_geometry COMMAND qbat_original_geometry_test)
  add_test(NAME qbat_original_source_identity COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_LIST_DIR}/source_fixture/verify_fixture.py")
endif()

option(QBAT_CUDA_CHECKS "Build root-scheduled actual QBAT device value checks" OFF)
if(QBAT_CUDA_CHECKS)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  add_executable(qbat_geometry_cuda_test MathCudaTest.cu)
  target_link_libraries(qbat_geometry_cuda_test PRIVATE tl_qbat_geometry GTest::gtest_main CUDA::cudart)
  set_target_properties(qbat_geometry_cuda_test PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
  target_compile_options(qbat_geometry_cuda_test PRIVATE
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
  add_test(NAME qbat_geometry_cuda COMMAND qbat_geometry_cuda_test)
  set_tests_properties(qbat_geometry_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 45)
endif()
