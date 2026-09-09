# Private engine context; the qualified startup library and receipt are unchanged.
option(T3_R2_BUILD_TESTS "Build bounded native T3 geometry/rate GTests" ON)
set(t3_engine_prepared "${CMAKE_CURRENT_BINARY_DIR}/engine-prepared")
set(t3_engine_modules "${CMAKE_CURRENT_BINARY_DIR}/engine-fortran-modules")
file(MAKE_DIRECTORY "${t3_engine_modules}")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --stage engine --output "${t3_engine_prepared}"
  RESULT_VARIABLE t3_engine_prepare_status ERROR_VARIABLE t3_engine_prepare_error)
if(NOT t3_engine_prepare_status EQUAL 0)
  message(FATAL_ERROR "Private T3 engine source preparation failed: ${t3_engine_prepare_error}")
endif()
add_custom_target(t3_r2_check_prepared
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
    --stage engine --output "${t3_engine_prepared}" --check VERBATIM)
set(t3_engine_sources T3NativeGeometry.F NativeT3Kinematics.F
  extracted/C3Coor3.F extracted/C3Evec3.F extracted/C3Deri3.F
  extracted/C3Defo3.F extracted/C3Curv3.F extracted/EngineSkew.F
  original/common_source/modules/constant_mod.F
  original/common_source/modules/precision_mod.F90
  original/common_source/modules/elements/element_mod.F90
  original/common_source/modules/mat_elem/elbufdef_mod.F90)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  T3NativeGeometry.F NativeT3Kinematics.F T3Engine.cmake
  extracted/C3Coor3.F extracted/C3Evec3.F extracted/C3Deri3.F
  extracted/C3Defo3.F extracted/C3Curv3.F)
list(TRANSFORM t3_engine_sources PREPEND "${t3_engine_prepared}/")
add_library(t3_r2_native STATIC T3Kinematics.cpp ${t3_engine_sources})
add_dependencies(t3_r2_native t3_r1_verify_sources t3_r2_check_prepared)
set_target_properties(t3_r2_native PROPERTIES Fortran_MODULE_DIRECTORY "${t3_engine_modules}")
target_compile_features(t3_r2_native PUBLIC cxx_std_17)
target_compile_options(t3_r2_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-openmp;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
target_compile_definitions(t3_r2_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:MYREAL8;CPP_mach=CPP_p4linux964;COMP_GFORTRAN>")
foreach(symbol C3COOR3 C3EVEC3 C3DERI3 C3DEFO3 C3CURV3 CLSKEW3
               COM08 COM08DP SCR05 IMPL1 IMPL1_PRIVATE)
  string(TOLOWER "${symbol}" lower)
  target_compile_definitions(t3_r2_native PRIVATE
    "$<$<COMPILE_LANGUAGE:Fortran>:${symbol}=T3_ENGINE_${symbol};${lower}=T3_ENGINE_${symbol}>")
endforeach()
target_include_directories(t3_r2_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${t3_tl_root}" PRIVATE
  "${t3_engine_modules}" "${t3_engine_prepared}/original/engine/share/spe_inc"
  "${t3_shared_original}/engine/share/spe_inc"
  "${t3_shared_original}/engine/share/includes"
  "${t3_shared_original}/engine/share/r8"
  "${t3_shared_original}/starter/share/includes")
target_link_libraries(t3_r2_native PUBLIC t3_r1_native PRIVATE Threads::Threads)
if(T3_R2_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(t3_kinematics_check T3KinematicsTest.cpp)
  target_link_libraries(t3_kinematics_check PRIVATE t3_r2_native GTest::gtest_main)
  target_compile_options(t3_kinematics_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME t3_kinematics_check COMMAND t3_kinematics_check)
  set_tests_properties(t3_kinematics_check PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endif()
