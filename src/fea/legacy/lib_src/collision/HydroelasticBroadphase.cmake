include_guard(GLOBAL)
get_filename_component(tl_broadphase_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${tl_broadphase_root}/lib_utils/CpuUtils.cmake")
find_package(CUDAToolkit REQUIRED)
add_library(tl_collision_broadphase STATIC
  "${CMAKE_CURRENT_LIST_DIR}/HydroelasticBroadphase.cu"
  "${tl_broadphase_root}/lib_utils/mesh_manager.cc")
target_include_directories(tl_collision_broadphase PUBLIC "${tl_broadphase_root}")
target_link_libraries(tl_collision_broadphase PUBLIC tl_cpu_utils CUDA::cudart)
set_target_properties(tl_collision_broadphase PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
