include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/PreparedCensusReplaySupport.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../../vehicle_run/RunReports.cmake")
add_library(robo_dyna_candidate_failure_fixture STATIC
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureFixture.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureExport.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureReplay.cpp")
target_link_libraries(robo_dyna_candidate_failure_fixture PUBLIC
  robo_dyna_self_contact_fixture_capture robo_dyna_prepared_census_replay
  robo_dyna_vehicle_run_reports)
target_compile_features(robo_dyna_candidate_failure_fixture PUBLIC cxx_std_17)
target_compile_options(robo_dyna_candidate_failure_fixture PRIVATE -fno-fast-math -ffp-contract=off)
