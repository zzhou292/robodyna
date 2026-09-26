include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25SearchStartup.cmake")
add_library(tl_radioss_type25_tied_removal STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/tied_removal/Admission.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/tied_removal/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/tied_removal/Values.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/tied_removal/Build.cpp")
target_link_libraries(tl_radioss_type25_tied_removal PUBLIC tl_radioss_type25_search_startup)
