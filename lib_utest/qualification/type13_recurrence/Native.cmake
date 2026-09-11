# Shared owning native library; callers explicitly enable Fortran.
include_guard(GLOBAL)
add_library(type13_h1_native STATIC
  "${CMAKE_CURRENT_LIST_DIR}/native/NativeChannel.F90"
  "${CMAKE_CURRENT_LIST_DIR}/native/NativeCurve.F"
  "${CMAKE_CURRENT_LIST_DIR}/native/NativeDeformation.F"
  "${CMAKE_CURRENT_LIST_DIR}/native/NativeFailure.F"
  "${CMAKE_CURRENT_LIST_DIR}/native/NativeStability.F"
  "${CMAKE_CURRENT_LIST_DIR}/../type25/native/NativeFrame.F"
  "${CMAKE_CURRENT_LIST_DIR}/../type25/native/NativeScatter.F")
target_include_directories(type13_h1_native PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/native"
  "${CMAKE_CURRENT_LIST_DIR}/../type25/native/stubs")
target_compile_options(type13_h1_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fno-fast-math -ffp-contract=off -fcheck=all -finit-real=snan)
