include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../VehiclePhysicalDynamics.cmake")
set(ROBO_DYNA_PHYSICAL_CAPTURE_RUNTIME ON)
include("${CMAKE_CURRENT_LIST_DIR}/../../../output/physical_frames/PhysicalFrames.cmake")
add_library(robo_dyna_vehicle_accepted_frames STATIC "${CMAKE_CURRENT_LIST_DIR}/VehicleAcceptedFrames.cpp")
target_link_libraries(robo_dyna_vehicle_accepted_frames PUBLIC robo_dyna_vehicle_physical_dynamics
    robo_dyna_physical_accepted_frames)
target_compile_features(robo_dyna_vehicle_accepted_frames PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_accepted_frames PRIVATE -fno-fast-math -ffp-contract=off)
