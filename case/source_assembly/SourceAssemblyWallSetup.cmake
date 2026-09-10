include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyBindings.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../shell_collection/ShellCollectionContactGeometry.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../PlacedCanonicalWall.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../wall_penalty/WallPenaltyCertification.cmake")
add_library(robo_dyna_source_assembly_wall_setup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyInitialKinetic.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallSetup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallDeviceConfig.cpp")
target_link_libraries(robo_dyna_source_assembly_wall_setup PUBLIC robo_dyna_source_assembly_bindings
  robo_dyna_shell_collection_contact_geometry robo_dyna_placed_canonical_wall robo_dyna_wall_penalty_certification)
target_compile_features(robo_dyna_source_assembly_wall_setup PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_wall_setup PRIVATE -fno-fast-math -ffp-contract=off)
# Config value declarations include CUDA's stream type, without a CUDA owner or
# runtime link. Integrated engine builds already provide these headers.
find_package(CUDAToolkit REQUIRED)
target_include_directories(robo_dyna_source_assembly_wall_setup PUBLIC "${CUDAToolkit_INCLUDE_DIRS}")
