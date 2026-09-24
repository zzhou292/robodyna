include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactFilterCertificates.cmake")
find_package(CUDAToolkit REQUIRED)
add_library(tl_self_contact_filter_batch_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_filters/Layout.cpp")
target_include_directories(tl_self_contact_filter_batch_values PUBLIC "${CUDAToolkit_INCLUDE_DIRS}")
target_link_libraries(tl_self_contact_filter_batch_values PUBLIC tl_self_contact_filter_certificates)
target_compile_features(tl_self_contact_filter_batch_values PUBLIC cxx_std_17)
target_compile_options(tl_self_contact_filter_batch_values PRIVATE -fno-fast-math -ffp-contract=off)
if(CMAKE_CUDA_COMPILER)
  add_library(tl_self_contact_filter_batch STATIC
    "${CMAKE_CURRENT_LIST_DIR}/self_contact_filters/Batch.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/self_contact_filters/Kernels.cu")
  target_link_libraries(tl_self_contact_filter_batch PUBLIC tl_self_contact_filter_batch_values CUDA::cudart)
  set_target_properties(tl_self_contact_filter_batch PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(tl_self_contact_filter_batch PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>)
endif()
