include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/ShellPhysicalOwnerValues.cmake")
add_library(tl_shell_physical_owner STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ShellPhysicalOwnerSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellPhysicalOwnerAssembly.cpp")
target_link_libraries(tl_shell_physical_owner PUBLIC tl_shell_physical_owner_values tl_explicit_nodal_state)
target_compile_features(tl_shell_physical_owner PUBLIC cxx_std_17)
target_compile_options(tl_shell_physical_owner PRIVATE -fno-fast-math -ffp-contract=off)
