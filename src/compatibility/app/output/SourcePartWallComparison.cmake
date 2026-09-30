include_guard(GLOBAL)
if(NOT TARGET robo_dyna_accepted_replay)
  message(FATAL_ERROR "Source wall comparison requires accepted replay")
endif()
add_library(robo_dyna_source_part_wall_comparison STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartWallComparison.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartWallComparisonInput.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourcePartWallComparisonIntervals.cpp")
target_link_libraries(robo_dyna_source_part_wall_comparison PUBLIC robo_dyna_accepted_replay)
target_compile_options(robo_dyna_source_part_wall_comparison PRIVATE -fno-fast-math -ffp-contract=off)
add_executable(robo_dyna_source_part_wall_compare "${CMAKE_CURRENT_LIST_DIR}/../case/source_part_wall/compare.cpp")
target_link_libraries(robo_dyna_source_part_wall_compare PRIVATE robo_dyna_source_part_wall_comparison)
if(ROBO_DYNA_ENABLE_REPLAY_CHECKS OR ROBO_DYNA_ENABLE_SOURCE_PART_ELASTIC)
  find_package(GTest REQUIRED)
  add_executable(robo_dyna_source_part_wall_comparison_check
    "${CMAKE_CURRENT_LIST_DIR}/source_part_wall_comparison_math_check.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/source_part_wall_comparison_events_check.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/source_part_wall_comparison_input_check.cpp")
  target_link_libraries(robo_dyna_source_part_wall_comparison_check PRIVATE
    robo_dyna_source_part_wall_comparison GTest::gtest_main)
  target_compile_options(robo_dyna_source_part_wall_comparison_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_part_wall_comparison COMMAND robo_dyna_source_part_wall_comparison_check)
  set_tests_properties(source_part_wall_comparison PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1)
endif()
