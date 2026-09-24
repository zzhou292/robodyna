include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RepresentedIntervalCrossing.cmake")
find_package(CUDAToolkit REQUIRED)
add_library(tl_represented_interval_crossing_gpu_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Forecast.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/OwnerStorage.h")
target_include_directories(tl_represented_interval_crossing_gpu_values PUBLIC "${CUDAToolkit_INCLUDE_DIRS}")
target_link_libraries(tl_represented_interval_crossing_gpu_values PUBLIC tl_represented_interval_crossing)
target_compile_features(tl_represented_interval_crossing_gpu_values PUBLIC cxx_std_17)
target_compile_options(tl_represented_interval_crossing_gpu_values PRIVATE -fno-fast-math -ffp-contract=off)
