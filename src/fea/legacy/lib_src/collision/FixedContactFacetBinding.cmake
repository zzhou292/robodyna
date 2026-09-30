include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactSurfaceBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/weighted_surface/WeightedSurface.cmake")
add_library(tl_fixed_contact_facets STATIC
  "${CMAKE_CURRENT_LIST_DIR}/FixedContactFacetBinding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact_facets/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact_facets/Templates.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact_facets/Keys.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/contact_facets/Approximation.cpp")
target_link_libraries(tl_fixed_contact_facets PUBLIC tl_self_contact_surface_binding tl_weighted_surface)
target_compile_features(tl_fixed_contact_facets PUBLIC cxx_std_17)
target_compile_options(tl_fixed_contact_facets PRIVATE -fno-fast-math -ffp-contract=off)
