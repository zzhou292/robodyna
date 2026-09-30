include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/Type25Model.cmake")
add_library(tl_type25_math INTERFACE)
target_link_libraries(tl_type25_math INTERFACE tl_type25_model)
