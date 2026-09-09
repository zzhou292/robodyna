# Optional integration checks for TL's complete prescribed Q4 operation.
# Links the actual owning coherent Chrono core; no source copying or rebuild.
enable_language(CUDA)
find_package(GTest REQUIRED)
set(CRASH_TL_FEA_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Total-Lagrangian-FEA"
    CACHE PATH "TL-FEA source root providing the CUDA element operations")
add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements" "tl-reissner-elements")
add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_utest/elements" "tl-reissner-tests")

add_library(robo_dyna_reissner_setup STATIC ReissnerShellSetup.cpp)
target_include_directories(robo_dyna_reissner_setup PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(robo_dyna_reissner_setup PUBLIC cxx_std_17)
target_compile_options(robo_dyna_reissner_setup PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(robo_dyna_reissner_setup PUBLIC tl_reissner_shell Chrono::Chrono_core)

function(robo_dyna_shell_check target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} PRIVATE robo_dyna_reissner_setup GTest::gtest_main)
  set_target_properties(${target} PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_definitions(${target} PRIVATE EIGEN_NO_CUDA)
  target_compile_options(${target} PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
  add_test(NAME ${target} COMMAND ${target})
  set_tests_properties(${target} PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endfunction()

robo_dyna_shell_check(robo_dyna_reissner_setup_check reissner_shell_setup_check.cpp)
robo_dyna_shell_check(robo_dyna_reissner_rotation_check reissner_rotation_cuda_check.cu)
robo_dyna_shell_check(robo_dyna_reissner_force_check reissner_shell_force_check.cu)
