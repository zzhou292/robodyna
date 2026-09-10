include_guard(GLOBAL)
add_library(robo_dyna_stage_timing STATIC "${CMAKE_CURRENT_LIST_DIR}/StageTimer.cpp")
target_include_directories(robo_dyna_stage_timing PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../..")
target_compile_features(robo_dyna_stage_timing PUBLIC cxx_std_17)
