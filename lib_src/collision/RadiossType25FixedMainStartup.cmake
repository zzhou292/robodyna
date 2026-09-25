include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Normal.cmake")
add_library(tl_radioss_type25_fixed_main_startup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/Checks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/Expand.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/Topology.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/OrderedNeighbors.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/References.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/Normals.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/SnapshotChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/Build.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/startup/FixedMain.cpp")
target_link_libraries(tl_radioss_type25_fixed_main_startup PUBLIC tl_radioss_type25_normal)
# Host startup only: no CUDA language/runtime, Fortran, GTest or physical owner.
