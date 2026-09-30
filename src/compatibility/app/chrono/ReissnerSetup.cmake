# Shared production setup adapter. Tests and case orchestration opt in separately.
set(CRASH_TL_FEA_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Total-Lagrangian-FEA"
    CACHE PATH "TL-FEA source root providing conventional shell operations")
add_subdirectory("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/elements" "tl-reissner-elements")
add_library(robo_dyna_reissner_setup STATIC ReissnerShellSetup.cpp)
target_include_directories(robo_dyna_reissner_setup PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(robo_dyna_reissner_setup PUBLIC cxx_std_17)
target_compile_options(robo_dyna_reissner_setup PRIVATE -fno-fast-math -ffp-contract=off)
target_link_libraries(robo_dyna_reissner_setup PUBLIC tl_reissner_shell Chrono::Chrono_core)
