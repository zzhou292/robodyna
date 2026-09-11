include_guard(GLOBAL)
find_package(CUDAToolkit REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../assembly/ShellPhysicalBinding.cmake")
add_library(tl_shell_physical_owner_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ShellPhysicalOwnerLayout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ShellPhysicalOutputRanges.cpp")
target_link_libraries(tl_shell_physical_owner_values PUBLIC tl_shell_physical_binding)
target_include_directories(tl_shell_physical_owner_values PUBLIC "${CUDAToolkit_INCLUDE_DIRS}")
target_compile_features(tl_shell_physical_owner_values PUBLIC cxx_std_17)
target_compile_options(tl_shell_physical_owner_values PRIVATE -fno-fast-math -ffp-contract=off)
