find_package(GTest REQUIRED)
add_executable(robo_dyna_physical_scene_check tests/ValuesTest.cpp tests/ActivityVisualTest.cpp tests/ArchiveSceneTest.cpp)
target_link_libraries(robo_dyna_physical_scene_check PRIVATE robo_dyna_physical_replay_scene GTest::gtest_main)
target_compile_options(robo_dyna_physical_scene_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME physical_scene_values COMMAND robo_dyna_physical_scene_check --gtest_filter=-PhysicalSceneArchive.*)
set_tests_properties(physical_scene_values PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)
set(ROBO_DYNA_PHYSICAL_REPLAY_INPUT "" CACHE FILEPATH "Explicit completed run viewer-input.json")
if(ROBO_DYNA_PHYSICAL_REPLAY_INPUT)
  add_test(NAME physical_scene_archive COMMAND robo_dyna_physical_scene_check --gtest_filter=PhysicalSceneArchive.*)
  set_tests_properties(physical_scene_archive PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "ROBO_DYNA_PHYSICAL_REPLAY_INPUT=${ROBO_DYNA_PHYSICAL_REPLAY_INPUT}")
endif()
add_executable(robo_dyna_physical_viewer_values_check tests/ViewerValuesTest.cpp tests/FixedCameraTest.cpp tests/ClippingTest.cpp)
target_link_libraries(robo_dyna_physical_viewer_values_check PRIVATE robo_dyna_physical_viewer_values GTest::gtest_main)
add_test(NAME physical_viewer_values COMMAND robo_dyna_physical_viewer_values_check)
set_tests_properties(physical_viewer_values PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1)
# Reuse the affected legacy scene/camera regressions in this small owning gate.
add_executable(robo_dyna_physical_scene_legacy_check
  "${CMAKE_CURRENT_LIST_DIR}/../../viewer/accepted_replay_scene_check.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../viewer/accepted_replay_view_check.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../viewer/accepted_replay_part_color_check.cpp")
target_link_libraries(robo_dyna_physical_scene_legacy_check PRIVATE robo_dyna_replay_scene GTest::gtest_main)
add_test(NAME physical_scene_legacy COMMAND robo_dyna_physical_scene_legacy_check)
set_tests_properties(physical_scene_legacy PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE PROCESSORS 1)

add_executable(robo_dyna_recovered_scene_check tests/RecoveredSceneTest.cpp)
target_link_libraries(robo_dyna_recovered_scene_check PRIVATE robo_dyna_physical_viewer_values GTest::gtest_main)
target_compile_options(robo_dyna_recovered_scene_check PRIVATE -fno-fast-math -ffp-contract=off)
set(ROBO_DYNA_RECOVERED_REPLAY_INPUT "" CACHE FILEPATH "Explicit recovered-samples.json descriptor")
if(ROBO_DYNA_RECOVERED_REPLAY_INPUT)
  add_test(NAME recovered_scene_archive COMMAND robo_dyna_recovered_scene_check)
  set_tests_properties(recovered_scene_archive PROPERTIES TIMEOUT 300 RUN_SERIAL TRUE PROCESSORS 1
    ENVIRONMENT "ROBO_DYNA_RECOVERED_REPLAY_INPUT=${ROBO_DYNA_RECOVERED_REPLAY_INPUT}")
endif()

# Explicit recovered declared-environment acceptance; never substitute an old
# no-environment archive or infer a normal run completion from saved samples.
add_executable(robo_dyna_recovered_environment_scene_check tests/RecoveredEnvironmentSceneTest.cpp)
target_link_libraries(robo_dyna_recovered_environment_scene_check PRIVATE robo_dyna_physical_viewer_values GTest::gtest_main)
target_compile_options(robo_dyna_recovered_environment_scene_check PRIVATE -fno-fast-math -ffp-contract=off)
