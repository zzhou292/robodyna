include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Coefficients.cmake")
add_library(tl_radioss_type25_nodal_seed STATIC
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_nodal/Build.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/source_nodal/Correction.cpp")
target_link_libraries(tl_radioss_type25_nodal_seed PUBLIC tl_radioss_type25_coefficients)
target_compile_features(tl_radioss_type25_nodal_seed PUBLIC cxx_std_17)
target_compile_options(tl_radioss_type25_nodal_seed PRIVATE -fno-fast-math -ffp-contract=off)
