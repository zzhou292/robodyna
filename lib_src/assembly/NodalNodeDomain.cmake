# Immutable declared node identity only; no coefficient or device owner.
include_guard(GLOBAL)
add_library(tl_nodal_node_domain STATIC "${CMAKE_CURRENT_LIST_DIR}/NodalNodeDomain.cpp")
target_include_directories(tl_nodal_node_domain PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../..")
target_compile_features(tl_nodal_node_domain PUBLIC cxx_std_17)
target_compile_options(tl_nodal_node_domain PRIVATE -fno-fast-math -ffp-contract=off)
