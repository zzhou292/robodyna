# Host-only immutable native reference/mass union; no batch, owner or oracle.
# Reuse the startup interfaces when a standalone batch composes only one family.
include_guard(GLOBAL)
foreach(family qeph t3)
  if(NOT TARGET tl_${family})
    add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/${family}"
      "${CMAKE_CURRENT_BINARY_DIR}/shell-binding-${family}")
  endif()
endforeach()
add_library(tl_shell_batch_binding STATIC "${CMAKE_CURRENT_LIST_DIR}/ShellBatchBinding.cpp")
target_link_libraries(tl_shell_batch_binding PUBLIC tl_qeph tl_t3)
target_compile_features(tl_shell_batch_binding PUBLIC cxx_std_17)
target_compile_options(tl_shell_batch_binding PRIVATE -fno-fast-math -ffp-contract=off)
