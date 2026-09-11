include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../../lib_src/constraints/tied_shell/TiedClassification.cmake")
find_package(GTest REQUIRED)
set(classification_native "${CMAKE_CURRENT_LIST_DIR}/native")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/classification_modules")
add_library(tied_classification_native STATIC
  "${classification_native}/Context.F90"
  "${classification_native}/NativeInterfaces.F"
  "${classification_native}/Packet.F"
  "${classification_native}/RigidPacket.F"
  "${classification_native}/Intab.F"
  "${classification_native}/original/starter/source/interfaces/inter3d1/itagsl2.F"
  "${classification_native}/original/starter/source/constraints/general/kinset.F"
  "${classification_native}/original/starter/source/constraints/general/kinini.F")
target_include_directories(tied_classification_native PRIVATE "${classification_native}/context"
  "${classification_native}" "${classification_native}/original/common_source/includes")
target_compile_options(tied_classification_native PRIVATE -cpp -ffixed-line-length-none
  -ffree-line-length-none -fcheck=all -fno-fast-math -ffp-contract=off)
set_target_properties(tied_classification_native PROPERTIES
  Fortran_MODULE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/classification_modules")
add_library(tied_classification_oracle STATIC "${CMAKE_CURRENT_LIST_DIR}/NativeOracle.cpp")
target_link_libraries(tied_classification_oracle PUBLIC tl_tied_shell_classification
  tied_classification_native GTest::gtest)
target_compile_options(tied_classification_oracle PRIVATE -fno-fast-math -ffp-contract=off)
