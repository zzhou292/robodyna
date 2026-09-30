include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/Type13Startup.cmake")
add_library(tl_type13_math INTERFACE)
target_link_libraries(tl_type13_math INTERFACE tl_type13_startup)
