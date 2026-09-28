include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../SourceAdmission.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25ActivitySource.cmake")
add_library(robo_dyna_native_activity_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Controls.cpp" "${CMAKE_CURRENT_LIST_DIR}/Declaration.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Coverage.cpp" "${CMAKE_CURRENT_LIST_DIR}/CoverageValues.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CoverageShells.cpp" "${CMAKE_CURRENT_LIST_DIR}/CoverageSolids.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CoverageBeams.cpp" "${CMAKE_CURRENT_LIST_DIR}/CoverageConnections.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Census.cpp" "${CMAKE_CURRENT_LIST_DIR}/Document.cpp")
target_link_libraries(robo_dyna_native_activity_source PUBLIC
  robo_dyna_native_vehicle_source_admission tl_radioss_type25_activity_source)
target_compile_features(robo_dyna_native_activity_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_activity_source PRIVATE -fno-fast-math -ffp-contract=off)
