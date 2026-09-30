include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SelfContactActiveUseBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/FixedTriangleFeatureDiscovery.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SurfaceJacobianMajorant.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/penalty_pair/SurfacePenaltyPair.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/ShellPhysicalOwner.cmake")

add_library(tl_self_contact_force_assembly STATIC
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_force/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_force/Values.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_force/Source.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_force/Initialize.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/self_contact_force/Operations.cu")
target_link_libraries(tl_self_contact_force_assembly PUBLIC
  tl_self_contact_active_uses
  tl_fixed_triangle_feature_discovery
  tl_surface_penalty_pair
  tl_surface_jacobian_majorant
  tl_shell_physical_owner)
target_compile_features(tl_self_contact_force_assembly PUBLIC cxx_std_17)
set_target_properties(tl_self_contact_force_assembly PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(tl_self_contact_force_assembly PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
