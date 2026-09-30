include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../modelio/self_contact/OriginalSelection.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactActiveUseBinding.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/SelfContactFilterCertificates.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/FixedTriangleFeatureDiscovery.cmake")
add_library(robo_dyna_vehicle_self_contact_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SelectedSelfContactSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/InitialCensusValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/InitialFeatureSampleValues.cpp")
target_link_libraries(robo_dyna_vehicle_self_contact_values PUBLIC
  robo_dyna_original_self_contact_selection
  tl_self_contact_active_uses tl_self_contact_filter_certificates
  tl_fixed_triangle_feature_discovery)
target_compile_features(robo_dyna_vehicle_self_contact_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_self_contact_values PRIVATE
  -fno-fast-math -ffp-contract=off)
