include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/VehicleContactStartup.cmake")
set(ROBO_DYNA_PHYSICAL_CAPTURE_RUNTIME ON)
set(ROBO_DYNA_PHYSICAL_RUN_LIVE_FACTORY ON)
set(ROBO_DYNA_PHYSICAL_RUN_ENVIRONMENT ON)
include("${CMAKE_CURRENT_LIST_DIR}/../../output/physical_frames/PhysicalFrames.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_run/RunReports.cmake")
add_library(robo_dyna_native_vehicle_run STATIC
  "${CMAKE_CURRENT_LIST_DIR}/run/Prepare.cpp" "${CMAKE_CURRENT_LIST_DIR}/run/Session.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/run/ArchiveRequest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../vehicle_dynamics/diagnostics/qeph_rejection/Codec.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../vehicle_dynamics/diagnostics/qeph_rejection/Export.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/run/Retry.cpp" "${CMAKE_CURRENT_LIST_DIR}/run/Execute.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/run/Summary.cpp")
target_link_libraries(robo_dyna_native_vehicle_run PUBLIC robo_dyna_vehicle_native_contact
  robo_dyna_vehicle_run_reports robo_dyna_physical_run_environment robo_dyna_physical_run_live)
target_compile_features(robo_dyna_native_vehicle_run PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_vehicle_run PRIVATE -fno-fast-math -ffp-contract=off)
