include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../elements/ShellPhysicalOwnerValues.cmake")
add_library(tl_self_contact_surface_binding STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SelfContactSurfaceBinding.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact/Sources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact/Features.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact/Queries.cpp")
target_link_libraries(tl_self_contact_surface_binding PUBLIC tl_shell_physical_owner_values)
target_compile_features(tl_self_contact_surface_binding PUBLIC cxx_std_17)
target_compile_options(tl_self_contact_surface_binding PRIVATE -fno-fast-math -ffp-contract=off)
