include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/CoverageFixtureCapture.cmake")

add_library(robo_dyna_prepared_census_replay STATIC
  "${CMAKE_CURRENT_LIST_DIR}/PreparedCensusReplay.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PreparedCensusReplayManifest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PreparedCensusReplayReport.cpp")
target_link_libraries(robo_dyna_prepared_census_replay PUBLIC
  robo_dyna_self_contact_fixture_capture robo_dyna_artifact_io)
target_compile_features(robo_dyna_prepared_census_replay PUBLIC cxx_std_17)
target_compile_options(robo_dyna_prepared_census_replay PRIVATE
  -fno-fast-math -ffp-contract=off)
