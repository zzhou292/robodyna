include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../../output/ArtifactIO.cmake")
add_library(robo_dyna_connectivity_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Components.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReportText.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Relations.cpp")
target_link_libraries(robo_dyna_connectivity_values PUBLIC robo_dyna_artifact_io)
target_include_directories(robo_dyna_connectivity_values PUBLIC "${ROBO_DYNA_TL_ROOT}")
target_compile_features(robo_dyna_connectivity_values PUBLIC cxx_std_17)
