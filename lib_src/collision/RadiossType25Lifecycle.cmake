include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Selection.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Coefficients.cmake")
add_library(tl_radioss_type25_lifecycle INTERFACE)
target_link_libraries(tl_radioss_type25_lifecycle INTERFACE
  tl_radioss_type25_selection tl_radioss_type25_coefficients)
