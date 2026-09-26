include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25InitialStateValues.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25TiedRemoval.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Candidates.cmake")
if(NOT CMAKE_CUDA_COMPILER)
  message(FATAL_ERROR "Native initial-state production requires its bounded GPU path")
endif()
find_package(CUDAToolkit REQUIRED)
set(_type25_initial_source "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/initial_source")
add_library(tl_radioss_type25_initial_state STATIC
  "${_type25_initial_source}/Admission.cpp" "${_type25_initial_source}/Host.cpp"
  "${_type25_initial_source}/Layout.cpp" "${_type25_initial_source}/Prepare.cpp"
  "${_type25_initial_source}/Operands.cu" "${_type25_initial_source}/Inventory.cu"
  "${_type25_initial_source}/Rows.cu")
target_link_libraries(tl_radioss_type25_initial_state PUBLIC tl_radioss_type25_initial_state_values
  tl_radioss_type25_tied_removal tl_radioss_type25_candidates CUDA::cudart)
set_target_properties(tl_radioss_type25_initial_state PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_radioss_type25_initial_state PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
