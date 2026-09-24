include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/FixedContactFacetBinding.cmake")
find_package(Threads REQUIRED)
add_library(tl_fixed_triangle_feature_discovery STATIC
  "${CMAKE_CURRENT_LIST_DIR}/fixed_triangle_features/ExactPredicates.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/fixed_triangle_features/ExactInteger.h"
  "${CMAKE_CURRENT_LIST_DIR}/fixed_triangle_features/ExactPredicateKernel.h"
  "${CMAKE_CURRENT_LIST_DIR}/fixed_triangle_features/Geometry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/fixed_triangle_features/Discovery.cpp")
target_link_libraries(tl_fixed_triangle_feature_discovery
  PUBLIC tl_fixed_contact_facets Threads::Threads)
target_compile_features(tl_fixed_triangle_feature_discovery PUBLIC cxx_std_17)
target_compile_options(tl_fixed_triangle_feature_discovery PRIVATE
  -fno-fast-math -ffp-contract=off)
