include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../benchmarks/stage_timing/StageTiming.cmake")
add_library(robo_dyna_source_assembly_pilot_options STATIC
  "${CMAKE_CURRENT_LIST_DIR}/PilotOptions.cpp" "${CMAKE_CURRENT_LIST_DIR}/CliOptions.cpp")
target_link_libraries(robo_dyna_source_assembly_pilot_options PUBLIC robo_dyna_stage_timing)
target_compile_features(robo_dyna_source_assembly_pilot_options PUBLIC cxx_std_17)
