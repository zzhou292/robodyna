include("${CMAKE_CURRENT_LIST_DIR}/../ReplayVsg.cmake")
add_executable(robo_dyna_physical_replay main.cpp Overlay.cpp Capture.cpp Metadata.cpp)
target_link_libraries(robo_dyna_physical_replay PRIVATE robo_dyna_physical_viewer_values robo_dyna_replay_vsg)
