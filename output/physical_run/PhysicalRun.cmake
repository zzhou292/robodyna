include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_frames/FrameArchive.cmake")
add_library(robo_dyna_physical_run_records STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Profile.cpp" "${CMAKE_CURRENT_LIST_DIR}/Sequence.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Fields.cpp" "${CMAKE_CURRENT_LIST_DIR}/IntervalWriter.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/IntervalReader.cpp" "${CMAKE_CURRENT_LIST_DIR}/Configuration.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/IndexFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/IndexChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Manifest.cpp" "${CMAKE_CURRENT_LIST_DIR}/Inventory.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReferencedInventory.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/WallFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/WallRead.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RunPrepare.cpp" "${CMAKE_CURRENT_LIST_DIR}/RunWrite.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Replay.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReplayRecords.cpp")
target_link_libraries(robo_dyna_physical_run_records PUBLIC robo_dyna_physical_frame_archive)
target_include_directories(robo_dyna_physical_run_records PRIVATE "${ROBO_DYNA_TL_ROOT}")
target_compile_features(robo_dyna_physical_run_records PUBLIC cxx_std_17)
target_compile_options(robo_dyna_physical_run_records PRIVATE -fno-fast-math -ffp-contract=off)
option(ROBO_DYNA_PHYSICAL_RUN_LIVE_FACTORY "Build accepted dynamics interval factory" OFF)
if(ROBO_DYNA_PHYSICAL_RUN_LIVE_FACTORY)
  include("${CMAKE_CURRENT_LIST_DIR}/../../case/vehicle_dynamics/output/VehicleAcceptedFrames.cmake")
  add_library(robo_dyna_physical_run_live STATIC "${CMAKE_CURRENT_LIST_DIR}/AcceptedDynamics.cpp")
  target_link_libraries(robo_dyna_physical_run_live PUBLIC robo_dyna_physical_run_records robo_dyna_vehicle_accepted_frames)
  target_compile_features(robo_dyna_physical_run_live PUBLIC cxx_std_17)
  target_compile_options(robo_dyna_physical_run_live PRIVATE -fno-fast-math -ffp-contract=off)
endif()
option(ROBO_DYNA_PHYSICAL_RUN_WALL "Build named actual vehicle wall archive adapter" OFF)
if(ROBO_DYNA_PHYSICAL_RUN_WALL)
  include("${CMAKE_CURRENT_LIST_DIR}/../../case/vehicle_wall/VehicleWallStartup.cmake")
  add_library(robo_dyna_physical_run_wall STATIC "${CMAKE_CURRENT_LIST_DIR}/WallPrepare.cpp")
  target_link_libraries(robo_dyna_physical_run_wall PUBLIC robo_dyna_physical_run_records
    robo_dyna_vehicle_wall_setup robo_dyna_physical_accepted_frames)
  target_compile_options(robo_dyna_physical_run_wall PRIVATE -fno-fast-math -ffp-contract=off)
endif()
