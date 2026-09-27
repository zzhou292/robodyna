include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_run/source/OriginalSourceIO.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/solid_control_packets/NativePacketSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../SourceAdmission.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_self_contact/native/InitializerControlsSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_self_contact/native/PostGapmMainSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../vehicle_startup/TiedSearchPostKinChk.cmake")
add_library(robo_dyna_native_v6_sources STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Inputs.cpp" "${CMAKE_CURRENT_LIST_DIR}/Owner.cpp" "${CMAKE_CURRENT_LIST_DIR}/Contact.cpp")
target_link_libraries(robo_dyna_native_v6_sources PUBLIC robo_dyna_original_source_io
  robo_dyna_native_solid_packets robo_dyna_native_vehicle_source_admission
  robo_dyna_initializer_controls_source robo_dyna_post_gapm_main_source robo_dyna_tied_search_post_kinchk)
target_compile_features(robo_dyna_native_v6_sources PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_v6_sources PRIVATE -fno-fast-math -ffp-contract=off)
