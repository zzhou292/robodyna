include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/TiedShellPacking.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../source_assembly/NativeCoordinates.cmake")
add_library(robo_dyna_tied_search_geometry STATIC
  "${CMAKE_CURRENT_LIST_DIR}/TiedShellSearchGeometry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/search_geometry/Build.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/search_geometry/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/search_geometry/Association.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/search_geometry/Properties.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/search_geometry/WorkingCoordinates.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/search_geometry/Coefficients.cpp")
target_link_libraries(robo_dyna_tied_search_geometry PUBLIC robo_dyna_tied_shell_packing robo_dyna_source_native_coordinates)
target_compile_features(robo_dyna_tied_search_geometry PUBLIC cxx_std_17)
target_compile_options(robo_dyna_tied_search_geometry PRIVATE -fno-fast-math -ffp-contract=off)
