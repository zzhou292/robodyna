include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactActiveUseBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/FixedTriangleFeatureDiscovery.cmake")

add_library(tl_self_contact_current_regularity STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SelfContactCurrentRegularity.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_current_regularity/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_current_regularity/Source.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_current_regularity/Values.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_current_regularity/Query.cpp")
target_link_libraries(tl_self_contact_current_regularity PUBLIC
  tl_self_contact_active_uses tl_fixed_triangle_feature_discovery)
target_compile_features(tl_self_contact_current_regularity PUBLIC cxx_std_17)
target_compile_options(tl_self_contact_current_regularity PRIVATE
  -fno-fast-math -ffp-contract=off)
