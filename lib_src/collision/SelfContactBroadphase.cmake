include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactSurfaceBinding.cmake")
find_package(CUDAToolkit REQUIRED)
add_library(tl_self_contact_broadphase_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_broadphase/Layout.cpp")
target_link_libraries(tl_self_contact_broadphase_values PUBLIC tl_self_contact_surface_binding CUDA::toolkit)
target_compile_features(tl_self_contact_broadphase_values PUBLIC cxx_std_17)
target_compile_options(tl_self_contact_broadphase_values PRIVATE -fno-fast-math -ffp-contract=off)
if(CMAKE_CUDA_COMPILER)
  add_library(tl_self_contact_broadphase STATIC "${CMAKE_CURRENT_LIST_DIR}/SelfContactBroadphase.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/self_contact_broadphase/Kernels.cu"
    "${CMAKE_CURRENT_LIST_DIR}/self_contact_broadphase/Sort.cu")
  target_link_libraries(tl_self_contact_broadphase PUBLIC tl_self_contact_broadphase_values CUDA::cudart)
  set_target_properties(tl_self_contact_broadphase PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(tl_self_contact_broadphase PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
endif()
