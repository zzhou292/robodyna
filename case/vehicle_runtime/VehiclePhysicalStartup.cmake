include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/shell_execution/VehicleShellExecution.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_startup/physical_attachments/VehiclePhysicalAttachments.cmake")
if(NOT TARGET tl_explicit_nodal_state)
  add_subdirectory("${ROBO_DYNA_TL_ROOT}/lib_src/solvers" "${CMAKE_CURRENT_BINARY_DIR}/physical-runtime-owner")
endif()
if(NOT TARGET tl_qeph_batch)
  include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/qeph/QephBatch.cmake")
endif()
if(NOT TARGET tl_t3_batch)
  include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/t3/T3Batch.cmake")
endif()
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchPublication.cmake")
add_library(robo_dyna_vehicle_runtime_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Config.cpp" "${CMAKE_CURRENT_LIST_DIR}/Packing.cpp")
target_link_libraries(robo_dyna_vehicle_runtime_values PUBLIC robo_dyna_artifact_io
  tl_nodal_rigid_assembly_binding CUDA::cudart)
target_include_directories(robo_dyna_vehicle_runtime_values PUBLIC "${ROBO_DYNA_TL_ROOT}")
target_compile_features(robo_dyna_vehicle_runtime_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_runtime_values PRIVATE -fno-fast-math -ffp-contract=off)
add_library(robo_dyna_vehicle_physical_startup STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehiclePhysicalStartup.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/CaptureAccess.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceIdentity.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceRoles.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceBudget.cpp" "${CMAKE_CURRENT_LIST_DIR}/Forecast.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ParticipantConfigs.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/InitializeOwner.cpp" "${CMAKE_CURRENT_LIST_DIR}/InitializeParticipants.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/InitializePublication.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/InspectOwner.cpp" "${CMAKE_CURRENT_LIST_DIR}/InspectShells.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/InspectConnections.cpp" "${CMAKE_CURRENT_LIST_DIR}/InspectSolids.cpp")
target_link_libraries(robo_dyna_vehicle_physical_startup PUBLIC robo_dyna_vehicle_runtime_values
  robo_dyna_vehicle_shell_execution robo_dyna_vehicle_physical_attachments tl_shell_batch_publication)
target_compile_features(robo_dyna_vehicle_physical_startup PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_physical_startup PRIVATE -fno-fast-math -ffp-contract=off)
