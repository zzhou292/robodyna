include_guard(GLOBAL)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT TARGET tied_search_native)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../tied_shell_search" "${CMAKE_CURRENT_BINARY_DIR}/search-primitives")
endif()
set(finalization_native "${CMAKE_CURRENT_LIST_DIR}/native")
set(classification_native "${CMAKE_CURRENT_LIST_DIR}/../tied_shell_classification/native")
include("${CMAKE_CURRENT_LIST_DIR}/../../../lib_src/constraints/tied_shell/search/TiedSearchFinalization.cmake")
get_target_property(finalization_shared_modules tied_search_native Fortran_MODULE_DIRECTORY)
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${finalization_native}/verify_sources.py" RESULT_VARIABLE identity)
if(NOT identity EQUAL 0)
  message(FATAL_ERROR "Native finalization source identity failed")
endif()
add_library(tied_search_finalization_native STATIC
  "${finalization_native}/Context.F90" "${finalization_native}/Interfaces.F90"
  "${finalization_native}/Storage.F90" "${finalization_native}/Packet.F90"
  "${finalization_native}/ConnectionCount.F"
  "${finalization_native}/original/starter/source/interfaces/inter3d1/i2tid3.F"
  "${classification_native}/original/starter/source/constraints/general/kinini.F"
  "${classification_native}/original/starter/source/constraints/general/kinset.F")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/finalization_modules")
set_target_properties(tied_search_finalization_native PROPERTIES Fortran_MODULE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/finalization_modules")
target_include_directories(tied_search_finalization_native PRIVATE "${finalization_native}/context" "${finalization_native}"
  "${classification_native}/original/common_source/includes" "${finalization_shared_modules}"
  "${CMAKE_CURRENT_BINARY_DIR}/finalization_modules")
target_compile_definitions(tied_search_finalization_native PRIVATE "my_real=double precision")
target_compile_options(tied_search_finalization_native PRIVATE -cpp -ffixed-line-length-none -ffree-line-length-none
  -fcheck=all -fno-fast-math -ffp-contract=off -finit-real=snan)
add_dependencies(tied_search_finalization_native tied_search_native)
target_link_libraries(tied_search_finalization_native PUBLIC tied_search_native)
add_library(tied_search_finalization_oracle STATIC "${CMAKE_CURRENT_LIST_DIR}/NativeOracle.cpp")
target_link_libraries(tied_search_finalization_oracle PUBLIC tl_tied_search_finalization tied_search_finalization_native GTest::gtest)
target_compile_options(tied_search_finalization_oracle PRIVATE -fno-fast-math -ffp-contract=off)
