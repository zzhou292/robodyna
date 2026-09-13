include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/FixedContactFacetBinding.cmake")
add_library(tl_represented_interval_crossing STATIC
  "${CMAKE_CURRENT_LIST_DIR}/RepresentedIntervalCrossing.cpp")
target_link_libraries(tl_represented_interval_crossing
  PUBLIC tl_fixed_contact_facets)
target_compile_features(tl_represented_interval_crossing PUBLIC cxx_std_17)
target_compile_options(tl_represented_interval_crossing
  PRIVATE -fno-fast-math -ffp-contract=off)
