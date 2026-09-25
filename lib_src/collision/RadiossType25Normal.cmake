include_guard(GLOBAL)
add_library(tl_radioss_type25_normal INTERFACE)
get_filename_component(_tl_type25_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(tl_radioss_type25_normal INTERFACE "${_tl_type25_root}")
target_compile_features(tl_radioss_type25_normal INTERFACE cxx_std_17)
# Production depends on headers/math only; never a native Fortran oracle.
