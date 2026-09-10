# Optional external-project viewer. Root enables CXX and adds output first;
# neither CUDA nor the TL/FEA/case targets are dependencies of these targets.
find_package(Chrono REQUIRED CONFIG)

add_library(robo_dyna_replay_scene STATIC ../chrono/AcceptedReplayScene.cpp ../chrono/ReplayPartColors.cpp)
target_include_directories(robo_dyna_replay_scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_compile_features(robo_dyna_replay_scene PUBLIC cxx_std_17)
target_link_libraries(robo_dyna_replay_scene PUBLIC Chrono::Chrono_core)
set_source_files_properties(../chrono/ReplayPartColors.cpp PROPERTIES
  COMPILE_OPTIONS "-fno-fast-math;-ffp-contract=off")

if(TARGET robo_dyna_artifact_io)
  add_library(robo_dyna_replay_color_metadata STATIC ReplayColorMetadata.cpp)
  target_link_libraries(robo_dyna_replay_color_metadata PUBLIC robo_dyna_replay_scene robo_dyna_artifact_io)
endif()

if(ROBO_DYNA_ENABLE_REPLAY)
  find_package(Chrono REQUIRED CONFIG COMPONENTS VSG)
  if(NOT TARGET Chrono::Chrono_vsg)
    message(FATAL_ERROR "Replay requires an external Chrono build with its VSG module enabled")
  endif()
  if(NOT TARGET robo_dyna_accepted_replay)
    message(FATAL_ERROR "Add the accepted replay reader target before viewer")
  endif()
  add_executable(robo_dyna_replay replay_main.cpp)
  target_compile_features(robo_dyna_replay PUBLIC cxx_std_17)
  target_link_libraries(robo_dyna_replay PRIVATE robo_dyna_replay_scene robo_dyna_accepted_replay
    robo_dyna_artifact_io robo_dyna_replay_color_metadata Chrono::Chrono_vsg)
  target_compile_definitions(robo_dyna_replay PRIVATE ROBO_DYNA_CHRONO_DATA_DIR="${CHRONO_DATA_DIR}")
endif()

option(ROBO_DYNA_ENABLE_REPLAY_SCENE_CHECKS "Build core-only accepted replay scene checks" OFF)
if(ROBO_DYNA_ENABLE_REPLAY_SCENE_CHECKS)
  find_package(GTest REQUIRED)
  add_executable(robo_dyna_replay_scene_check accepted_replay_scene_check.cpp accepted_replay_view_check.cpp
    accepted_replay_part_color_check.cpp)
  target_link_libraries(robo_dyna_replay_scene_check PRIVATE robo_dyna_replay_scene GTest::gtest_main)
  if(TARGET robo_dyna_accepted_replay)
    target_sources(robo_dyna_replay_scene_check PRIVATE accepted_plastic_replay_check.cpp)
    target_link_libraries(robo_dyna_replay_scene_check PRIVATE robo_dyna_accepted_replay)
  endif()
  if(TARGET robo_dyna_replay_color_metadata)
    target_sources(robo_dyna_replay_scene_check PRIVATE accepted_replay_color_metadata_check.cpp)
    target_link_libraries(robo_dyna_replay_scene_check PRIVATE robo_dyna_replay_color_metadata)
  endif()
  add_test(NAME accepted_replay_scene COMMAND robo_dyna_replay_scene_check)
  set_tests_properties(accepted_replay_scene PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
endif()
