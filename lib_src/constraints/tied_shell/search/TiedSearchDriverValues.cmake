include_guard(GLOBAL)
get_filename_component(tl_driver_root "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include("${CMAKE_CURRENT_LIST_DIR}/../TiedSearch.cmake")
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(CUDAToolkit REQUIRED)
add_library(tl_tied_search_driver_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Checks.cpp" "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Projection.cpp")
target_include_directories(tl_tied_search_driver_values PUBLIC "${tl_driver_root}")
target_link_libraries(tl_tied_search_driver_values PUBLIC tl_tied_shell_search Eigen3::Eigen CUDA::cudart)
target_compile_features(tl_tied_search_driver_values PUBLIC cxx_std_17)
target_compile_options(tl_tied_search_driver_values PRIVATE -fno-fast-math -ffp-contract=off)
