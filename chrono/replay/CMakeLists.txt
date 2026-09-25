add_library(robo_dyna_replay_geometry_values STATIC ../ReplayPartColors.cpp ../ReplayDisplayGeometry.cpp)
target_include_directories(robo_dyna_replay_geometry_values PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../..")
target_compile_features(robo_dyna_replay_geometry_values PUBLIC cxx_std_17)
target_link_libraries(robo_dyna_replay_geometry_values PUBLIC Chrono::Chrono_core)
set_source_files_properties(../ReplayPartColors.cpp PROPERTIES
  COMPILE_OPTIONS "-fno-fast-math;-ffp-contract=off")

add_library(robo_dyna_replay_visual_values STATIC ../ReplayVisuals.cpp ../ReplayCamera.cpp ../ReplayClipping.cpp)
target_link_libraries(robo_dyna_replay_visual_values PUBLIC robo_dyna_replay_geometry_values)
add_library(robo_dyna_replay_scene STATIC ../AcceptedReplayScene.cpp)
target_include_directories(robo_dyna_replay_scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../..")
target_compile_features(robo_dyna_replay_scene PUBLIC cxx_std_17)
target_link_libraries(robo_dyna_replay_scene PUBLIC robo_dyna_replay_visual_values)
