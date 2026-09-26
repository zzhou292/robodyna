include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../modelio/native_contact_scene/DeclaredSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/tied_shell/search/TiedSearchFinalization.cmake")
add_library(robo_dyna_native_scene_tied_source STATIC "${CMAKE_CURRENT_LIST_DIR}/TiedSource.cpp")
target_link_libraries(robo_dyna_native_scene_tied_source PUBLIC robo_dyna_native_scene_declared
  tl_tied_cin_attachment tl_tied_search_finalization)
target_compile_options(robo_dyna_native_scene_tied_source PRIVATE -fno-fast-math -ffp-contract=off)
