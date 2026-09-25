# Native numerical inventory; production has no Fortran/oracle dependency.
if(NOT TARGET tl_radioss_type25_candidate_values)
  get_filename_component(_type25_candidates_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
  add_library(tl_radioss_type25_candidate_values INTERFACE)
  target_include_directories(tl_radioss_type25_candidate_values INTERFACE "${_type25_candidates_root}")
  target_compile_features(tl_radioss_type25_candidate_values INTERFACE cxx_std_17)
  target_compile_options(tl_radioss_type25_candidate_values INTERFACE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--ftz=false;--prec-div=true;--prec-sqrt=true>")
endif()
if(CMAKE_CUDA_COMPILER AND NOT TARGET tl_radioss_type25_candidates)
  find_package(CUDAToolkit REQUIRED)
  set(_type25_candidates "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/candidates")
  add_library(tl_radioss_type25_candidates STATIC
    "${_type25_candidates}/Layout.cpp" "${_type25_candidates}/Initialize.cpp"
    "${_type25_candidates}/Inventory.cpp" "${_type25_candidates}/Kernels.cu")
  target_link_libraries(tl_radioss_type25_candidates PUBLIC tl_radioss_type25_candidate_values CUDA::cudart)
  set_target_properties(tl_radioss_type25_candidates PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
endif()
