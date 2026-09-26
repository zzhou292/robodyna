# Native TYPE25 GPU runtime. No native oracle or executable is a production dependency.
include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Lifecycle.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Candidates.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Search.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25Assembly.cmake")
if(NOT TARGET tl_shell_batch_publication OR NOT TARGET tl_radioss_type25_search)
  message(FATAL_ERROR "TYPE25 transaction requires the physical owner/publication and TYPE25_SEARCH_CUDA")
endif()
set(_type25_runtime "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/runtime")
add_library(tl_radioss_type25_transaction STATIC
  "${_type25_runtime}/physical_main/Index.cpp"
  "${_type25_runtime}/physical_main/Origins.cpp"
  "${_type25_runtime}/physical_main/Validate.cpp"
  "${_type25_runtime}/Layout.cpp" "${_type25_runtime}/Source.cpp" "${_type25_runtime}/MovingSource.cpp" "${_type25_runtime}/MixedSource.cpp"
  "${_type25_runtime}/Initialize.cpp" "${_type25_runtime}/Transaction.cpp"
  "${_type25_runtime}/Kernels.cu" "${_type25_runtime}/NormalStage.cu")
target_link_libraries(tl_radioss_type25_transaction PUBLIC
  tl_radioss_type25_lifecycle tl_radioss_type25_candidates tl_radioss_type25_search
  tl_radioss_type25_assembly_device tl_shell_batch_publication CUDA::cudart)
set_target_properties(tl_radioss_type25_transaction PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_radioss_type25_transaction PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
