include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/NodalWallContactDevice.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/ShellBatchPublication.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/ShellPhysicalOwner.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/RigidNormalResponse.cmake")
add_library(tl_nodal_wall_mapped STATIC
  "${CMAKE_CURRENT_LIST_DIR}/nodal_wall_mapped/Layout.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_wall_mapped/Forecast.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_wall_mapped/Sources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_wall_mapped/Activity.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_wall_mapped/Initialize.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_wall_mapped/Readback.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/nodal_wall_mapped/Operations.cu")
target_compile_features(tl_nodal_wall_mapped PUBLIC cxx_std_17)
set_target_properties(tl_nodal_wall_mapped PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_link_libraries(tl_nodal_wall_mapped PUBLIC tl_nodal_wall_contact_device
  tl_shell_batch_publication tl_shell_physical_owner tl_rigid_normal_response)
target_compile_options(tl_nodal_wall_mapped PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
