include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/InitialSurfaceSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25InterfaceSurface.cmake")
add_library(robo_dyna_mixed_interface_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/MixedInterfaceSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_interface/Packing.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_interface/Certificate.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_interface/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_interface/Census.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_interface/Digest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/mixed_interface/Document.cpp")
target_link_libraries(robo_dyna_mixed_interface_source PUBLIC
  robo_dyna_initial_surface_source tl_radioss_type25_interface_surface)
target_compile_features(robo_dyna_mixed_interface_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_mixed_interface_source PRIVATE -fno-fast-math -ffp-contract=off)
