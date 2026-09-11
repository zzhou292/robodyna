include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../Model.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../ShellPhysicalOwnerValues.cmake")
add_library(tl_type45_batch_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Arena.cpp" "${CMAKE_CURRENT_LIST_DIR}/Plan.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Forecast.cpp" "${CMAKE_CURRENT_LIST_DIR}/Upload.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Identity.cpp" "${CMAKE_CURRENT_LIST_DIR}/OutputRanges.cpp")
target_link_libraries(tl_type45_batch_values PUBLIC tl_type45_model tl_shell_physical_owner_values)
target_compile_features(tl_type45_batch_values PUBLIC cxx_std_17)
target_compile_options(tl_type45_batch_values PRIVATE -fno-fast-math -ffp-contract=off)
