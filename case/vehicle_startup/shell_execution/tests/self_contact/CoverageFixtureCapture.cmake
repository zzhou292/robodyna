include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../../../vehicle_self_contact/VehicleSelfContactRuntime.cmake")

add_library(robo_dyna_self_contact_fixture_capture STATIC
  "${CMAKE_CURRENT_LIST_DIR}/CandidateCapture.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CoverageFixtureCapture.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CoverageFixtureGeometry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CoverageFixtureInspection.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CensusFixtureExport.cpp")
target_link_libraries(robo_dyna_self_contact_fixture_capture PUBLIC
  robo_dyna_vehicle_self_contact_runtime
  tl_self_contact_transaction tl_fixed_triangle_feature_discovery)
target_compile_features(robo_dyna_self_contact_fixture_capture PUBLIC cxx_std_17)
target_compile_options(robo_dyna_self_contact_fixture_capture PRIVATE
  -fno-fast-math -ffp-contract=off)
