# Compose this contribution with the caller's single explicit nodal owner.
if(NOT TARGET tl_q4_planar_contact)
  if(NOT TARGET tl_explicit_nodal_state)
    message(FATAL_ERROR "Add TL lib_src/solvers before Q4 planar contact")
  endif()
  include("${CMAKE_CURRENT_LIST_DIR}/Q4PlanarGeometry.cmake")
  add_library(tl_q4_planar_contact STATIC
    "${CMAKE_CURRENT_LIST_DIR}/Q4PlanarContact.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Q4PlanarContactDevice.cu")
  target_compile_features(tl_q4_planar_contact PUBLIC cxx_std_17)
  set_target_properties(tl_q4_planar_contact PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_link_libraries(tl_q4_planar_contact PUBLIC tl_explicit_nodal_state tl_q4_planar_geometry)
  target_compile_options(tl_q4_planar_contact PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
endif()
