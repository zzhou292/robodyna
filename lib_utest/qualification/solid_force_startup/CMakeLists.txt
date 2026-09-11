cmake_minimum_required(VERSION 3.22)
project(TLSolidForceStartup LANGUAGES CXX C)
get_filename_component(TL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
enable_testing()
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_executable(solid_force_startup_host HostTest.cpp)
target_compile_features(solid_force_startup_host PRIVATE cxx_std_17)
target_include_directories(solid_force_startup_host PRIVATE "${TL_ROOT}")
target_compile_options(solid_force_startup_host PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(solid_force_startup_host PRIVATE GTest::gtest_main Eigen3::Eigen)
add_test(NAME solid_force_startup_host COMMAND solid_force_startup_host)
add_executable(solid_force_startup_legacy_host
  ../solid18_force/HostTest.cpp ../solid24_force/HostTest.cpp
  ../solid6z_force/HostTest.cpp ../solid_law36_point/HostTest.cpp
  ../solid_law42_caller/HostTest.cpp)
target_compile_features(solid_force_startup_legacy_host PRIVATE cxx_std_17)
target_include_directories(solid_force_startup_legacy_host PRIVATE "${TL_ROOT}")
target_compile_options(solid_force_startup_legacy_host PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(solid_force_startup_legacy_host PRIVATE GTest::gtest_main Eigen3::Eigen)
add_test(NAME solid_force_startup_legacy_host COMMAND solid_force_startup_legacy_host)
option(TL_SOLID_FORCE_STARTUP_NATIVE "Build independent native TT0 and source gates" OFF)
option(TL_SOLID_FORCE_STARTUP_CUDA "Build device-owned startup/retry versus native gate" OFF)
if(TL_SOLID_FORCE_STARTUP_CUDA AND NOT TL_SOLID_FORCE_STARTUP_NATIVE)
  message(FATAL_ERROR "Solid startup CUDA requires the independent native oracle")
endif()
if(TL_SOLID_FORCE_STARTUP_NATIVE)
  enable_language(Fortran)
  add_subdirectory(../solid_law36_point/native law36_native)
  add_subdirectory(../solid_law42_point/native law42_native)
  add_subdirectory(../solid_law42_caller/native law42_caller_native)
  add_subdirectory(../solid18_reference/native reference18_native)
  add_subdirectory(../solid24_reference/native reference24_native)
  add_subdirectory(../solid6z_reference/native reference6z_native)
  add_subdirectory(../solid18_force/native force18_native)
  add_subdirectory(../solid24_force/native force24_native)
  add_subdirectory(../solid6z_force/native force6z_native)
  add_library(solid_startup_oracles STATIC
    ../solid18_force/NativeOracle.cpp ../solid18_force/PacketChecks.cpp
    ../solid18_reference/NativeOracle.cpp)
  target_compile_features(solid_startup_oracles PUBLIC cxx_std_17)
  target_include_directories(solid_startup_oracles PUBLIC "${TL_ROOT}")
  target_compile_options(solid_startup_oracles PRIVATE -fno-fast-math -ffp-contract=off)
  target_link_libraries(solid_startup_oracles PUBLIC GTest::gtest Eigen3::Eigen
    solid18_force_native solid18_reference_native heph_force_native
    solid24_reference_native solid6z_force_native solid6z_reference_native)
  set(stiffness_prepared "${CMAKE_CURRENT_BINARY_DIR}/stiffness_prepared")
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/native/prepare_sources.py" --output "${stiffness_prepared}"
    RESULT_VARIABLE source_status)
  if(NOT source_status EQUAL 0)
    message(FATAL_ERROR "Solid startup native stiffness source identity failed")
  endif()
  add_library(solid_startup_stiffness_native STATIC native/NativeStiffness.F90)
  add_custom_target(solid_startup_verify_stiffness COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/native/prepare_sources.py" --output "${stiffness_prepared}" --check)
  add_dependencies(solid_startup_stiffness_native law42_solid_caller_native solid_startup_verify_stiffness)
  target_include_directories(solid_startup_stiffness_native PRIVATE "${stiffness_prepared}"
    "${CMAKE_CURRENT_BINARY_DIR}/law42_caller_native/modules")
  target_compile_options(solid_startup_stiffness_native PRIVATE -cpp -fcheck=bounds
    -fbacktrace -fno-fast-math -ffp-contract=off)
  target_link_libraries(solid_startup_stiffness_native PUBLIC law42_solid_caller_native)
  add_test(NAME solid_startup_stiffness_identity COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/native/prepare_sources.py" --output "${stiffness_prepared}" --check)
  add_executable(solid_force_startup_native Native18Test.cpp Native24Test.cpp Native6zTest.cpp
    StiffnessNativeTest.cpp)
  target_compile_options(solid_force_startup_native PRIVATE -fno-fast-math -ffp-contract=off)
  target_link_libraries(solid_force_startup_native PRIVATE solid_startup_oracles
    solid_startup_stiffness_native GTest::gtest_main)
  add_test(NAME solid_force_startup_native COMMAND solid_force_startup_native --gtest_filter=SolidStartupNative.*)
  add_test(NAME solid_force_startup_source COMMAND solid_force_startup_native --gtest_filter=SolidStartupSource.*)
  add_executable(solid_force_startup_legacy_native
    ../solid18_force/NativeTest.cpp ../solid24_force/NativeTest.cpp ../solid6z_force/NativeTest.cpp)
  target_compile_options(solid_force_startup_legacy_native PRIVATE -fno-fast-math -ffp-contract=off)
  target_link_libraries(solid_force_startup_legacy_native PRIVATE solid_startup_oracles GTest::gtest_main)
  add_test(NAME solid_force_startup_legacy_native COMMAND solid_force_startup_legacy_native)
endif()
if(TL_SOLID_FORCE_STARTUP_CUDA)
  enable_language(CUDA)
  add_executable(solid_force_startup_cuda CudaTest.cu)
  target_compile_features(solid_force_startup_cuda PRIVATE cuda_std_17)
  target_compile_options(solid_force_startup_cuda PRIVATE --fmad=false --prec-div=true
    --prec-sqrt=true -Xcompiler=-fno-fast-math,-ffp-contract=off)
  target_link_libraries(solid_force_startup_cuda PRIVATE solid_startup_oracles GTest::gtest_main)
  add_test(NAME solid_force_startup_cuda COMMAND solid_force_startup_cuda)
endif()
get_property(tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${tests} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 180)
