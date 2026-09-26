include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/CorrectedNodalSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/CoatedSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25SurfaceSource.cmake")
add_library(robo_dyna_initial_surface_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/InitialSurfaceSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/solid_surfaces/Packing.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/solid_surfaces/Certificate.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/solid_surfaces/Context.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/solid_surfaces/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/solid_surfaces/Digest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/solid_surfaces/Document.cpp")
target_link_libraries(robo_dyna_initial_surface_source PUBLIC robo_dyna_vehicle_corrected_nodal_source
  robo_dyna_v5_coated_source tl_radioss_type25_surface_source)
target_compile_features(robo_dyna_initial_surface_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_initial_surface_source PRIVATE -fno-fast-math -ffp-contract=off)
