# Prescribed discrete reference-measure contact; no mechanics owner or CUDA state.
if(NOT TARGET tl_q4_parametric_contact)
  include("${CMAKE_CURRENT_LIST_DIR}/SurfaceMaterialMeasure.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/PlanarWallGeometry.cmake")
  add_library(tl_q4_parametric_contact STATIC "${CMAKE_CURRENT_LIST_DIR}/Q4ParametricContact.cpp")
  target_compile_features(tl_q4_parametric_contact PUBLIC cxx_std_17)
  target_compile_options(tl_q4_parametric_contact PRIVATE -fno-fast-math -ffp-contract=off)
  target_link_libraries(tl_q4_parametric_contact PUBLIC tl_surface_material_measure tl_planar_wall_geometry)
endif()
