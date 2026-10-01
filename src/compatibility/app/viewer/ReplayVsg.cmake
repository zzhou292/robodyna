include_guard(GLOBAL)
find_package(vsg REQUIRED CONFIG)
find_package(vsgXchange REQUIRED CONFIG)
find_package(vsgImGui REQUIRED CONFIG)
find_package(Chrono REQUIRED CONFIG COMPONENTS VSG)
add_library(robo_dyna_replay_vsg STATIC "${CMAKE_CURRENT_LIST_DIR}/ReplayVsg.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReplayVsgCapture.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReplayAssets.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/VsgImageCapture.cpp")
target_include_directories(robo_dyna_replay_vsg PUBLIC "${CMAKE_CURRENT_LIST_DIR}/..")
target_compile_features(robo_dyna_replay_vsg PUBLIC cxx_std_17)
target_compile_definitions(robo_dyna_replay_vsg PRIVATE ROBO_DYNA_CHRONO_DATA_DIR="${CHRONO_DATA_DIR}")
target_link_libraries(robo_dyna_replay_vsg PUBLIC Chrono::Chrono_vsg robo_dyna_artifact_io robo_dyna_replay_visual_values)
