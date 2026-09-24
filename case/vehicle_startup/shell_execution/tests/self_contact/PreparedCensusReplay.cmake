include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/PreparedCensusReplaySupport.cmake")

add_executable(robo_dyna_prepared_census_replay_tool
  "${CMAKE_CURRENT_LIST_DIR}/PreparedCensusReplayMain.cpp")
target_link_libraries(robo_dyna_prepared_census_replay_tool PRIVATE
  robo_dyna_prepared_census_replay)

add_executable(robo_dyna_prepared_census_replay_check
  "${CMAKE_CURRENT_LIST_DIR}/PreparedCensusReplayTest.cpp")
target_link_libraries(robo_dyna_prepared_census_replay_check PRIVATE
  robo_dyna_prepared_census_replay GTest::gtest_main)
target_compile_options(robo_dyna_prepared_census_replay_check PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_prepared_census_replay
  COMMAND robo_dyna_prepared_census_replay_check --gtest_filter=PreparedCensusReplay.*)
set_tests_properties(vehicle_prepared_census_replay PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1 LABELS "unit;self-contact;fixture-replay;host")
