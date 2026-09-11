include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../physical_model/VehiclePhysicalModel.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../../modelio/type45/VehicleType45Source.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/type45/Model.cmake")
add_library(robo_dyna_vehicle_joint_model STATIC "${CMAKE_CURRENT_LIST_DIR}/VehicleJointModel.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Budget.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceMapping.cpp")
target_link_libraries(robo_dyna_vehicle_joint_model PUBLIC robo_dyna_vehicle_physical_model
    robo_dyna_vehicle_type45_source tl_type45_model)
target_compile_features(robo_dyna_vehicle_joint_model PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_joint_model PRIVATE -fno-fast-math -ffp-contract=off)
