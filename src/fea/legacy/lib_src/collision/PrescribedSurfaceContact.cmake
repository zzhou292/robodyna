# Mixed native-parent composition shares geometry and raw integrals, with no state owner.
if(NOT TARGET tl_prescribed_surface_contact)
  include("${CMAKE_CURRENT_LIST_DIR}/SurfaceMaterialMeasure.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/PlanarWallGeometry.cmake")
  add_library(tl_prescribed_surface_contact STATIC "${CMAKE_CURRENT_LIST_DIR}/PrescribedSurfaceContact.cpp")
  target_compile_features(tl_prescribed_surface_contact PUBLIC cxx_std_17)
  target_compile_options(tl_prescribed_surface_contact PRIVATE -fno-fast-math -ffp-contract=off)
  target_link_libraries(tl_prescribed_surface_contact PUBLIC tl_surface_material_measure tl_planar_wall_geometry)
endif()
