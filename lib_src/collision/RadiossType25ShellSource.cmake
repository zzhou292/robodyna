include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Coefficients.cmake")
add_library(tl_radioss_type25_shell_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_shells/Admission.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_shells/Build.cpp")
target_link_libraries(tl_radioss_type25_shell_source PUBLIC tl_radioss_type25_coefficients)
