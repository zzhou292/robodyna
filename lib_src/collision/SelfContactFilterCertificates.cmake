include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/FixedContactFacetBinding.cmake")
add_library(tl_self_contact_filter_certificates STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SelfContactFilterCertificates.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_filters/PrismQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_filters/Arithmetic.h"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_filters/Prism.h")
target_link_libraries(tl_self_contact_filter_certificates
  PUBLIC tl_fixed_contact_facets)
target_compile_features(
  tl_self_contact_filter_certificates PUBLIC cxx_std_17)
target_compile_options(
  tl_self_contact_filter_certificates PRIVATE
  -fno-fast-math -ffp-contract=off)
