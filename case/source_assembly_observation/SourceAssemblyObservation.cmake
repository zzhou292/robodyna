include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../source_assembly/SourceAssemblyBindings.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/constraints/NodalRigidObservation.cmake")
find_package(CUDAToolkit REQUIRED)
add_library(robo_dyna_source_assembly_observation STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyKinetic.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyKick.cpp")
target_link_libraries(robo_dyna_source_assembly_observation PUBLIC
  robo_dyna_source_assembly_bindings tl_nodal_rigid_observation CUDA::cudart)
target_compile_features(robo_dyna_source_assembly_observation PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_observation PRIVATE -fno-fast-math -ffp-contract=off)
