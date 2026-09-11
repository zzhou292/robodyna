include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../../lib_src/constraints/tied_shell/TiedPatch.cmake")
find_package(GTest REQUIRED)
add_library(tied_patch_native STATIC ${CMAKE_CURRENT_LIST_DIR}/native/NativeForce.F ${CMAKE_CURRENT_LIST_DIR}/native/NativeMotion.F ${CMAKE_CURRENT_LIST_DIR}/native/NativeMotionPacket.F90
  ${CMAKE_CURRENT_LIST_DIR}/native/NativeCoefficients.F ${CMAKE_CURRENT_LIST_DIR}/native/NativeAssembly.F)
target_include_directories(tied_patch_native PRIVATE "${CMAKE_CURRENT_LIST_DIR}/native" "${CMAKE_CURRENT_LIST_DIR}/native/stubs")
target_compile_options(tied_patch_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fno-fast-math -ffp-contract=off -fcheck=all -finit-real=snan)
add_library(tied_patch_oracle STATIC "${CMAKE_CURRENT_LIST_DIR}/NativeOracle.cpp")
target_link_libraries(tied_patch_oracle PUBLIC tl_tied_shell_patch tied_patch_native GTest::gtest)
target_compile_options(tied_patch_oracle PRIVATE -fno-fast-math -ffp-contract=off)
