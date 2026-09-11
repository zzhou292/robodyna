include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/Type13Startup.cmake")
add_library(tl_type13_model STATIC)
target_sources(tl_type13_model PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/Type13Model.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Type13ModelChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Type13ModelIdentity.cpp")
target_link_libraries(tl_type13_model PUBLIC tl_type13_startup)
target_compile_options(tl_type13_model PRIVATE -fno-fast-math -ffp-contract=off)
