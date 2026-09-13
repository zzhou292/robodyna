include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/FixedContactFacetBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SurfaceMaterialMeasure.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../constraints/NodalRigidAssemblyBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../constraints/tied_shell/TiedCinAttachment.cmake")
add_library(tl_self_contact_active_uses STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SelfContactActiveUseBinding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_active_use/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_active_use/Build.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_active_use/Queries.cpp")
target_link_libraries(tl_self_contact_active_uses PUBLIC
  tl_fixed_contact_facets tl_surface_material_measure
  tl_nodal_rigid_assembly_binding tl_tied_cin_attachment)
target_compile_features(tl_self_contact_active_uses PUBLIC cxx_std_17)
target_compile_options(tl_self_contact_active_uses PRIVATE
  -fno-fast-math -ffp-contract=off)
