# Immutable composition only; no CUDA allocation, force or dynamics ownership.
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../elements/ShellBatchBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/type25/Type25Model.cmake")
add_library(tl_nodal_mass_binding STATIC "${CMAKE_CURRENT_LIST_DIR}/NodalMassBinding.cpp")
target_link_libraries(tl_nodal_mass_binding PUBLIC tl_shell_batch_binding tl_type25_model)
target_compile_features(tl_nodal_mass_binding PUBLIC cxx_std_17)
target_compile_options(tl_nodal_mass_binding PRIVATE -fno-fast-math -ffp-contract=off)
