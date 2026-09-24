include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RepresentedIntervalCrossing.cmake")
get_property(_native_gpu_languages GLOBAL PROPERTY ENABLED_LANGUAGES)
if(NOT "CUDA" IN_LIST _native_gpu_languages)
  enable_language(CUDA)
endif()
unset(_native_gpu_languages)
find_package(CUDAToolkit REQUIRED)
add_library(tl_represented_interval_crossing_gpu STATIC
  "${CMAKE_CURRENT_LIST_DIR}/RepresentedIntervalCrossingGpu.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RepresentedIntervalCrossingGpu.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/DeviceExecution.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/DeviceBatch.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Workspace.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Cohort.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Transport.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Kernels.cu"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Workspace.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/KernelTypes.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native_device/Resources.h")
target_link_libraries(tl_represented_interval_crossing_gpu
  PUBLIC tl_represented_interval_crossing CUDA::cudart)
target_compile_features(tl_represented_interval_crossing_gpu PUBLIC cxx_std_17)
set_target_properties(tl_represented_interval_crossing_gpu PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_represented_interval_crossing_gpu PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;--expt-relaxed-constexpr;-Xcompiler=-fno-fast-math,-ffp-contract=off;--resource-usage>")
