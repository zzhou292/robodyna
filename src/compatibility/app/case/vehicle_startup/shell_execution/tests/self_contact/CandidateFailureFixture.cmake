include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/CandidateFailureSupport.cmake")

add_executable(robo_dyna_candidate_failure_fixture_check
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureFixtureTest.cpp")
target_link_libraries(robo_dyna_candidate_failure_fixture_check PRIVATE
  robo_dyna_candidate_failure_fixture GTest::gtest_main)
target_compile_options(robo_dyna_candidate_failure_fixture_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_candidate_failure_fixture_values
  COMMAND robo_dyna_candidate_failure_fixture_check --gtest_filter=CandidateFailureFixtureValues.*)
set_tests_properties(vehicle_candidate_failure_fixture_values PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1 LABELS "unit;host;failure-fixture")

add_executable(robo_dyna_candidate_failure_native_check
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureNativeTest.cpp")
target_link_libraries(robo_dyna_candidate_failure_native_check PRIVATE
  robo_dyna_candidate_failure_fixture tl_represented_interval_crossing GTest::gtest_main)
target_compile_options(robo_dyna_candidate_failure_native_check PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_candidate_failure_native_replay
  COMMAND robo_dyna_candidate_failure_native_check
    --gtest_filter=CandidateFailureNativeReplay.PinnedAffineLocalPairRetainsNativeWholeIntervalProof)
set_tests_properties(vehicle_candidate_failure_native_replay PROPERTIES
  TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1 LABELS "unit;host;failure-fixture;caller-pinned")


add_executable(robo_dyna_candidate_failure_cone_check
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureConeTest.cpp")
target_link_libraries(robo_dyna_candidate_failure_cone_check PRIVATE
  robo_dyna_candidate_failure_fixture tl_represented_interval_crossing GTest::gtest_main)
target_compile_options(robo_dyna_candidate_failure_cone_check PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_candidate_failure_cone_replay
  COMMAND robo_dyna_candidate_failure_cone_check
    --gtest_filter=CandidateFailureConeReplay.PinnedAffineSharedVertexPairRetainsCompleteLocalPolicy)
set_tests_properties(vehicle_candidate_failure_cone_replay PROPERTIES
  TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1 LABELS "unit;host;failure-fixture;caller-pinned")

add_executable(robo_dyna_candidate_failure_cone_search_check
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureConeSearchTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CandidateFailureConeAssertions.h")
target_link_libraries(robo_dyna_candidate_failure_cone_search_check PRIVATE
  robo_dyna_candidate_failure_fixture tl_represented_interval_crossing GTest::gtest_main)
target_compile_options(robo_dyna_candidate_failure_cone_search_check PRIVATE
  -fno-fast-math -ffp-contract=off)
add_test(NAME vehicle_candidate_failure_cone_search_replay
  COMMAND robo_dyna_candidate_failure_cone_search_check
    --gtest_filter=CandidateFailureConeSearchReplay.EpochOneAffinePairNeedsGenericSearchAndCompleteLocalPolicy)
set_tests_properties(vehicle_candidate_failure_cone_search_replay PROPERTIES
  TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1 LABELS "unit;host;failure-fixture;caller-pinned")
