include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Coefficients.cmake")
add_library(tl_radioss_type25_main_geometry INTERFACE)
target_link_libraries(tl_radioss_type25_main_geometry INTERFACE tl_radioss_type25_coefficients)
# Precision usage requirements inherited; production has no native oracle linkage.
