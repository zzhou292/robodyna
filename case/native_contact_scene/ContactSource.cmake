include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/PhysicalSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25ShellSource.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25FixedMainStartup.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25SearchStartup.cmake")
add_library(robo_dyna_native_scene_contact_source STATIC "${CMAKE_CURRENT_LIST_DIR}/ContactSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ContactPrepare.cpp" "${CMAKE_CURRENT_LIST_DIR}/ContactInputs.cpp" "${CMAKE_CURRENT_LIST_DIR}/ContactLayout.cpp")
target_link_libraries(robo_dyna_native_scene_contact_source PUBLIC robo_dyna_native_scene_physical_source
  tl_radioss_type25_shell_source tl_radioss_type25_fixed_main_startup tl_radioss_type25_search_startup)
target_compile_options(robo_dyna_native_scene_contact_source PRIVATE -fno-fast-math -ffp-contract=off)
