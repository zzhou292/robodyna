include_guard(GLOBAL)
find_package(CUDAToolkit REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/physical_attachments/VehiclePhysicalAttachments.cmake")
add_library(robo_dyna_vehicle_runtime_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Config.cpp" "${CMAKE_CURRENT_LIST_DIR}/Packing.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceRoles.cpp")
target_link_libraries(robo_dyna_vehicle_runtime_values PUBLIC robo_dyna_artifact_io
  tl_nodal_rigid_assembly_binding robo_dyna_vehicle_physical_attachments CUDA::cudart)
target_include_directories(robo_dyna_vehicle_runtime_values PUBLIC "${ROBO_DYNA_TL_ROOT}")
target_compile_features(robo_dyna_vehicle_runtime_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_runtime_values PRIVATE -fno-fast-math -ffp-contract=off)
