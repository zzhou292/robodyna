include_guard(GLOBAL)
find_package(GTest REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(bucket_tl_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
if(NOT TARGET tied_search_native)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../tied_shell_search"
    "${CMAKE_CURRENT_BINARY_DIR}/search-primitives")
endif()
set(bucket_shared "${CMAKE_CURRENT_LIST_DIR}/../native/qeph/original")
set(bucket_search "${CMAKE_CURRENT_LIST_DIR}/../tied_shell_search/native")
get_target_property(bucket_shared_modules tied_search_native Fortran_MODULE_DIRECTORY)
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/native/verify_sources.py"
  RESULT_VARIABLE bucket_identity)
if(NOT bucket_identity EQUAL 0)
  message(FATAL_ERROR "Native bucket source identity failed")
endif()
add_library(tied_search_bucket_native STATIC
  "${CMAKE_CURRENT_LIST_DIR}/native/Context.F90"
  "${CMAKE_CURRENT_LIST_DIR}/native/Domain.F"
  "${CMAKE_CURRENT_LIST_DIR}/native/Coordinates.F"
  "${CMAKE_CURRENT_LIST_DIR}/native/Packet.F90"
  "${bucket_shared}/common_source/modules/precision_mod.F90"
  "${bucket_shared}/common_source/modules/elements/element_mod.F90"
  "${bucket_search}/original/starter/source/interfaces/inter3d1/i2trivox.F90")
set_target_properties(tied_search_bucket_native PROPERTIES
  Fortran_MODULE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bucket_modules")
target_include_directories(tied_search_bucket_native PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/native" "${bucket_search}" "${bucket_shared_modules}"
  "${CMAKE_CURRENT_BINARY_DIR}/bucket_modules" "${bucket_shared}/engine/share/spe_inc")
target_compile_definitions(tied_search_bucket_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
target_compile_options(tied_search_bucket_native PRIVATE -cpp -ffixed-line-length-none
  -ffree-line-length-none -fno-fast-math -ffp-contract=off -fcheck=all -finit-real=snan)
add_dependencies(tied_search_bucket_native tied_search_native)
target_link_libraries(tied_search_bucket_native PUBLIC tied_search_native)
