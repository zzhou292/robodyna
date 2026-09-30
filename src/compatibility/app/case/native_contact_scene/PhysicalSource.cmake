include_guard(GLOBAL)
get_filename_component(robo_scene_case_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${robo_scene_case_root}/modelio/native_contact_scene/DeclaredSource.cmake")
include("${robo_scene_case_root}/modelio/native_contact_scene/MaterialBridge.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/assembly/ShellPhysicalBinding.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/tied_shell/TiedCinAttachment.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/TiedSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/TiedCinWitnessRoster.cmake")
add_library(robo_dyna_native_scene_physical_source STATIC "${CMAKE_CURRENT_LIST_DIR}/PhysicalSource.cpp")
target_link_libraries(robo_dyna_native_scene_physical_source PUBLIC robo_dyna_native_scene_declared
  robo_dyna_native_scene_material tl_shell_physical_binding tl_tied_cin_attachment robo_dyna_native_scene_tied_source
  robo_dyna_tied_cin_witness_roster)
target_compile_options(robo_dyna_native_scene_physical_source PRIVATE -fno-fast-math -ffp-contract=off)
