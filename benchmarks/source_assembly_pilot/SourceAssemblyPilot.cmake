include_guard(GLOBAL)
if(NOT TARGET robo_dyna_accepted_replay)
  message(FATAL_ERROR "Source assembly pilot comparison requires the owning accepted replay target")
endif()
add_library(robo_dyna_source_assembly_pilot STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Comparison.cpp" "${CMAKE_CURRENT_LIST_DIR}/ComparisonInput.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Metrics.cpp" "${CMAKE_CURRENT_LIST_DIR}/FieldDifferences.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RunSummary.cpp")
target_link_libraries(robo_dyna_source_assembly_pilot PUBLIC robo_dyna_accepted_replay)
target_compile_features(robo_dyna_source_assembly_pilot PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_pilot PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_assembly_pilot_compare "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries(robo_dyna_source_assembly_pilot_compare PRIVATE robo_dyna_source_assembly_pilot)
