# Prescribed force composition is separate from geometry preparation and from
# the CUDA owner-bound contributor. Reuse the same host geometry library.
if(NOT TARGET tl_q4_prescribed_planar_contact)
  include("${CMAKE_CURRENT_LIST_DIR}/Q4PlanarGeometry.cmake")
  add_library(tl_q4_prescribed_planar_contact STATIC "${CMAKE_CURRENT_LIST_DIR}/Q4PrescribedPlanarContact.cpp")
  target_compile_features(tl_q4_prescribed_planar_contact PUBLIC cxx_std_17)
  target_compile_options(tl_q4_prescribed_planar_contact PRIVATE -fno-fast-math -ffp-contract=off)
  target_link_libraries(tl_q4_prescribed_planar_contact PUBLIC tl_q4_planar_geometry)
endif()
