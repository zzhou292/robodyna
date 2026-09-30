# Distinct prescribed nodal penalty model; no owner or finite-wall admission.
if(NOT TARGET tl_nodal_wall_contact)
  include("${CMAKE_CURRENT_LIST_DIR}/Q4ParametricContact.cmake")
  add_library(tl_nodal_wall_contact STATIC
    "${CMAKE_CURRENT_LIST_DIR}/NodalWallContact.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/NodalWallWeightStartup.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/NodalWallContactEvaluation.cpp")
  target_compile_features(tl_nodal_wall_contact PUBLIC cxx_std_17)
  target_compile_options(tl_nodal_wall_contact PRIVATE -fno-fast-math -ffp-contract=off)
  target_link_libraries(tl_nodal_wall_contact PUBLIC tl_q4_parametric_contact)
endif()
