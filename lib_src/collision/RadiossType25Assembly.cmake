include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Friction.cmake")
add_library(tl_radioss_type25_assembly INTERFACE)
target_link_libraries(tl_radioss_type25_assembly INTERFACE tl_radioss_type25_friction)
if(CMAKE_CUDA_COMPILER)
  find_package(CUDAToolkit REQUIRED)
  set(_type25_assembly_device "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/assembly/device")
  add_library(tl_radioss_type25_assembly_device STATIC
    "${_type25_assembly_device}/Layout.cpp" "${_type25_assembly_device}/Initialize.cpp"
    "${_type25_assembly_device}/Stage.cpp" "${_type25_assembly_device}/Kernels.cu")
  target_link_libraries(tl_radioss_type25_assembly_device PUBLIC tl_radioss_type25_assembly CUDA::cudart)
  set_target_properties(tl_radioss_type25_assembly_device PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
endif()
