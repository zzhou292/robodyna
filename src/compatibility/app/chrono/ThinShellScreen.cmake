# Optional host-only reference utilities. The caller supplies the existing
# parameterized coupon reference and GTest; no CUDA owner, dynamics or default
# production inertia policy is changed. The later screen coordinator is separate.
add_library(robo_dyna_shell_patch_inertia STATIC ShellPatchInertia.cpp)
target_link_libraries(robo_dyna_shell_patch_inertia PUBLIC robo_dyna_elastic_coupon_reference)
target_compile_options(robo_dyna_shell_patch_inertia PRIVATE -fno-fast-math -ffp-contract=off)

add_library(robo_dyna_shell_mode_comparison STATIC ShellModeComparison.cpp)
target_link_libraries(robo_dyna_shell_mode_comparison PUBLIC robo_dyna_elastic_coupon_reference)
target_compile_options(robo_dyna_shell_mode_comparison PRIVATE -fno-fast-math -ffp-contract=off)

foreach(utility IN ITEMS shell_patch_inertia shell_mode_comparison)
  add_executable(robo_dyna_${utility}_check ${utility}_check.cpp)
  target_link_libraries(robo_dyna_${utility}_check PRIVATE robo_dyna_${utility} GTest::gtest_main)
  target_compile_options(robo_dyna_${utility}_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME ${utility} COMMAND robo_dyna_${utility}_check)
  set_tests_properties(${utility} PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endforeach()

add_library(robo_dyna_thin_shell_screen STATIC
  ThinShellScreen.cpp ThinShellScreenSpectrum.cpp ThinShellScreenModes.cpp)
target_link_libraries(robo_dyna_thin_shell_screen PUBLIC
  robo_dyna_shell_patch_inertia robo_dyna_shell_mode_comparison)
target_compile_options(robo_dyna_thin_shell_screen PRIVATE -fno-fast-math -ffp-contract=off)

add_library(robo_dyna_thin_shell_screen_report STATIC ThinShellScreenReport.cpp ThinShellScreenReportModes.cpp)
target_link_libraries(robo_dyna_thin_shell_screen_report PUBLIC robo_dyna_thin_shell_screen robo_dyna_artifact_io)
target_compile_options(robo_dyna_thin_shell_screen_report PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo-dyna-thin-shell-screen thin_shell_screen_main.cpp)
target_link_libraries(robo-dyna-thin-shell-screen PRIVATE robo_dyna_thin_shell_screen_report)

foreach(check IN ITEMS thin_shell_screen thin_shell_screen_report)
  add_executable(robo_dyna_${check}_check ${check}_check.cpp)
  target_link_libraries(robo_dyna_${check}_check PRIVATE robo_dyna_${check} GTest::gtest_main)
  target_compile_options(robo_dyna_${check}_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME ${check} COMMAND robo_dyna_${check}_check)
  set_tests_properties(${check} PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
endforeach()
