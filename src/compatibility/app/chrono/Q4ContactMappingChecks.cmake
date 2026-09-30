find_package(GTest REQUIRED)
set(CRASH_TL_FEA_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Total-Lagrangian-FEA"
    CACHE PATH "TL-FEA source root providing the checked Q4 mapping")
add_executable(robo_dyna_q4_surface_mapping_check q4_surface_mapping_check.cpp)
target_include_directories(robo_dyna_q4_surface_mapping_check PRIVATE "${CRASH_TL_FEA_SOURCE_DIR}")
target_compile_features(robo_dyna_q4_surface_mapping_check PRIVATE cxx_std_17)
target_compile_options(robo_dyna_q4_surface_mapping_check PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(robo_dyna_q4_surface_mapping_check PRIVATE Chrono::Chrono_core GTest::gtest_main)
add_test(NAME q4_surface_mapping_chrono COMMAND robo_dyna_q4_surface_mapping_check)
set_tests_properties(q4_surface_mapping_chrono PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
