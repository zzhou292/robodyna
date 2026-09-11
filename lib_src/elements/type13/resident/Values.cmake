include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../Type13Math.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../assembly/Type13NodeContributions.cmake")
find_package(CUDAToolkit REQUIRED)
add_library(tl_type13_batch_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Arena.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Forecast.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Startup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Identity.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/OutputRanges.cpp")
target_link_libraries(tl_type13_batch_values PUBLIC tl_type13_math tl_type13_node_contributions)
target_include_directories(tl_type13_batch_values PUBLIC ${CUDAToolkit_INCLUDE_DIRS})
target_compile_options(tl_type13_batch_values PRIVATE -fno-fast-math -ffp-contract=off)
