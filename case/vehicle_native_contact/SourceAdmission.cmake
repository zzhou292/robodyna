include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_wall/native/FiniteWallContactSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_self_contact/native/MixedStarterSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_self_contact/native/InitializerControlsSource.cmake")
add_library(robo_dyna_native_vehicle_source_admission STATIC "${CMAKE_CURRENT_LIST_DIR}/SourceAdmission.cpp")
target_link_libraries(robo_dyna_native_vehicle_source_admission PUBLIC
  robo_dyna_finite_wall_contact_source robo_dyna_mixed_starter_source robo_dyna_initializer_controls_source)
target_compile_features(robo_dyna_native_vehicle_source_admission PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_vehicle_source_admission PRIVATE -fno-fast-math -ffp-contract=off)
