include_guard(GLOBAL)
add_library(tl_radioss_type25_normal INTERFACE)
get_filename_component(_tl_type25_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(tl_radioss_type25_normal INTERFACE "${_tl_type25_root}")
target_compile_features(tl_radioss_type25_normal INTERFACE cxx_std_17)
# Production depends on headers/math only; never a native Fortran oracle.

# Precision is a usage requirement for every consumer of this header-only law.
# Qualified toolchains are GNU C++ and NVIDIA CUDA; callers must not override it.
target_compile_options(tl_radioss_type25_normal INTERFACE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANG_AND_ID:CUDA,NVIDIA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
