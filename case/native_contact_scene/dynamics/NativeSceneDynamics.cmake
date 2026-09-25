include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../ContactSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../ArchiveSource.cmake")
if(NOT TARGET robo_dyna_native_accepted_frames)
  message(FATAL_ERROR "Native scene dynamics requires ROBO_DYNA_NATIVE_CAPTURE_RUNTIME")
endif()
# Reuse the existing physical ledger packing implementation, without creating
# the vehicle startup class or importing qualification source data.
add_library(robo_dyna_native_scene_owner_packing STATIC "${robo_scene_case_root}/case/vehicle_runtime/Packing.cpp")
target_link_libraries(robo_dyna_native_scene_owner_packing PUBLIC robo_dyna_native_scene_physical_source CUDA::cudart Eigen3::Eigen)
add_library(robo_dyna_native_scene_dynamics STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Configuration.cpp" "${CMAKE_CURRENT_LIST_DIR}/Forecast.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Initialize.cpp" "${CMAKE_CURRENT_LIST_DIR}/NativeSceneDynamics.cpp")
target_link_libraries(robo_dyna_native_scene_dynamics PUBLIC robo_dyna_native_scene_contact_source
  robo_dyna_native_accepted_frames robo_dyna_native_scene_owner_packing)
include("${robo_scene_case_root}/case/vehicle_run/Values.cmake")
add_library(robo_dyna_native_scene_run STATIC "${CMAKE_CURRENT_LIST_DIR}/RunPlan.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RunExecute.cpp" "${CMAKE_CURRENT_LIST_DIR}/RunSummary.cpp")
target_link_libraries(robo_dyna_native_scene_run PUBLIC robo_dyna_native_scene_dynamics
  robo_dyna_native_scene_archive_source robo_dyna_vehicle_run_values)
add_library(robo_dyna_native_scene_options STATIC "${CMAKE_CURRENT_LIST_DIR}/Options.cpp")
target_link_libraries(robo_dyna_native_scene_options PUBLIC robo_dyna_native_scene_run)
foreach(t robo_dyna_native_scene_owner_packing robo_dyna_native_scene_dynamics robo_dyna_native_scene_run robo_dyna_native_scene_options)
  target_compile_options(${t} PRIVATE -fno-fast-math -ffp-contract=off)
endforeach()
add_executable(robo_dyna_native_scene_run_cli "${CMAKE_CURRENT_LIST_DIR}/Main.cpp")
set_target_properties(robo_dyna_native_scene_run_cli PROPERTIES OUTPUT_NAME robo_dyna_native_scene_run)
target_link_libraries(robo_dyna_native_scene_run_cli PRIVATE robo_dyna_native_scene_options)
