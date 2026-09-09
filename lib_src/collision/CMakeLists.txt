# Opt-in finite planar mesh batch. The parent composes the real nodal target;
# no DEME, donor qualification or second dynamics backend is pulled in.
if(NOT TARGET tl_explicit_nodal_state)
  message(FATAL_ERROR "Add TL lib_src/solvers before the planar contact module")
endif()
add_library(tl_planar_mesh_contact STATIC PlanarMeshContact.cpp PlanarMeshContactDevice.cu)
target_compile_features(tl_planar_mesh_contact PUBLIC cxx_std_17)
set_target_properties(tl_planar_mesh_contact PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_link_libraries(tl_planar_mesh_contact PUBLIC tl_explicit_nodal_state)
target_compile_options(tl_planar_mesh_contact PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false>")
