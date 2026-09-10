# Focused production target for conventional physical-node dynamics. The caller
# explicitly enables CUDA; no tests, donor qualification, DEME or viewer is
# pulled into this library. Bazel remains the repository-wide build.
find_package(CUDAToolkit REQUIRED)
get_filename_component(TL_NODAL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include("${CMAKE_CURRENT_LIST_DIR}/../constraints/NodalRigidGroup.cmake")
add_library(tl_explicit_nodal_state STATIC FENodalState.cu ExplicitTranslationStep.cu ExplicitNodalStep.cu
  NodalRigidGroupStorage.cu ExplicitNodalRigidStep.cu)
target_compile_features(tl_explicit_nodal_state PUBLIC cxx_std_17)
set_target_properties(tl_explicit_nodal_state PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_include_directories(tl_explicit_nodal_state PUBLIC "${TL_NODAL_ROOT}")
target_link_libraries(tl_explicit_nodal_state PUBLIC CUDA::cudart tl_nodal_rigid_group)
target_compile_options(tl_explicit_nodal_state PRIVATE
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
