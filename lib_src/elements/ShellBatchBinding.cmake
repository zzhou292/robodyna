# Host-only immutable native reference/mass union; no batch, owner or oracle.
# The composing project provides the existing tl_qeph and tl_t3 interfaces.
include_guard(GLOBAL)
add_library(tl_shell_batch_binding STATIC "${CMAKE_CURRENT_LIST_DIR}/ShellBatchBinding.cpp")
target_link_libraries(tl_shell_batch_binding PUBLIC tl_qeph tl_t3)
target_compile_features(tl_shell_batch_binding PUBLIC cxx_std_17)
target_compile_options(tl_shell_batch_binding PRIVATE -fno-fast-math -ffp-contract=off)
