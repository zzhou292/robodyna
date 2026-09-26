include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Coefficients.cmake")
add_library(tl_radioss_type25_gap_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_gaps/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_gaps/Rows.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_gaps/Build.cpp")
target_link_libraries(tl_radioss_type25_gap_source PUBLIC tl_radioss_type25_coefficients)
