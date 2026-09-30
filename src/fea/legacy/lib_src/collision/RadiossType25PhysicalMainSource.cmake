# Shared host-only physical contact-source identity validation.
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../assembly/ShellPhysicalBinding.cmake")
set(_type25_physical_main "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/runtime/physical_main")
add_library(tl_radioss_type25_physical_main_source STATIC
  "${_type25_physical_main}/Index.cpp" "${_type25_physical_main}/Origins.cpp" "${_type25_physical_main}/Validate.cpp")
target_link_libraries(tl_radioss_type25_physical_main_source PUBLIC tl_shell_physical_binding)
target_compile_features(tl_radioss_type25_physical_main_source PUBLIC cxx_std_17)
target_compile_options(tl_radioss_type25_physical_main_source PRIVATE -fno-fast-math -ffp-contract=off)
