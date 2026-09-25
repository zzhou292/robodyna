include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25FixedMainStartup.cmake")
add_library(tl_radioss_type25_search_startup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/search_startup/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/search_startup/Admission.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/search_startup/Multiplier.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/search_startup/Margin.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/search_startup/Curvature.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/search_startup/Removal.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/search_startup/Build.cpp")
target_link_libraries(tl_radioss_type25_search_startup PUBLIC tl_radioss_type25_fixed_main_startup)
