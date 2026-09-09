# Stateless finite-wall contributor on the existing nodal owner.
include_guard(GLOBAL)
if(NOT TARGET tl_explicit_nodal_state)
  message(FATAL_ERROR "Add TL lib_src/solvers before nodal-wall contact")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/NodalWallContact.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/PreparedPlanarWallQuery.cmake")
add_library(tl_nodal_wall_contact_device STATIC
  "${CMAKE_CURRENT_LIST_DIR}/NodalWallContactModel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NodalWallContactDevice.cu")
target_compile_features(tl_nodal_wall_contact_device PUBLIC cxx_std_17)
set_target_properties(tl_nodal_wall_contact_device PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_link_libraries(tl_nodal_wall_contact_device PUBLIC
  tl_explicit_nodal_state tl_nodal_wall_contact tl_prepared_planar_wall_query)
target_compile_options(tl_nodal_wall_contact_device PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
