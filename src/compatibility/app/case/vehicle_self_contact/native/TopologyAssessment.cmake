include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/self_contact/OriginalSelection.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25FixedMainStartup.cmake")
add_library(robo_dyna_native_topology_assessment STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TopologyInputs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/TopologyAssessment.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/TopologyDigest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/AssessmentDocument.cpp")
target_link_libraries(robo_dyna_native_topology_assessment PUBLIC
  robo_dyna_original_self_contact_selection tl_radioss_type25_fixed_main_startup)
target_compile_features(robo_dyna_native_topology_assessment PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_topology_assessment PRIVATE -fno-fast-math -ffp-contract=off)
