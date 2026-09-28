include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25ActivitySource.cmake")
set(_type25_activity_operands "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/activity_operands")
add_library(tl_radioss_type25_activity_operands STATIC
  "${_type25_activity_operands}/Admission.cpp" "${_type25_activity_operands}/Layout.cpp"
  "${_type25_activity_operands}/Startup.cpp" "${_type25_activity_operands}/Stage.cpp"
  "${_type25_activity_operands}/Kernels.cu")
target_link_libraries(tl_radioss_type25_activity_operands PUBLIC tl_radioss_type25_activity_source CUDA::cudart)
target_compile_features(tl_radioss_type25_activity_operands PUBLIC cxx_std_17)
set_target_properties(tl_radioss_type25_activity_operands PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_radioss_type25_activity_operands PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
