include_guard(GLOBAL)
# Public Settings/RuntimeBudget headers expose the real FENodalState CUDA ABI.
find_package(CUDAToolkit REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../../output/ArtifactIO.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/PlanarWallGeometry.cmake")
add_library(robo_dyna_vehicle_wall_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Settings.cpp" "${CMAKE_CURRENT_LIST_DIR}/EnvelopeWall.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RuntimeBudget.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/loaded/Config.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../wall_penalty/WallPlacementBounds.cpp")
get_filename_component(vehicle_wall_app_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(robo_dyna_vehicle_wall_values PUBLIC "${vehicle_wall_app_root}"
  "${ROBO_DYNA_TL_ROOT}" "${ROBO_DYNA_TL_ROOT}/lib_src" "${CUDAToolkit_INCLUDE_DIRS}")
target_link_libraries(robo_dyna_vehicle_wall_values PUBLIC robo_dyna_artifact_io tl_planar_wall_geometry CUDA::cudart)
target_compile_features(robo_dyna_vehicle_wall_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_wall_values PRIVATE -fno-fast-math -ffp-contract=off)
