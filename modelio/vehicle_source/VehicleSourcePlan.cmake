include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../source_assembly/Reader.cmake")
if(NOT TARGET robo_dyna_full_shell_source_mapping)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../output/full_shell" "${CMAKE_CURRENT_BINARY_DIR}/vehicle_source_io")
endif()
add_library(robo_dyna_vehicle_source STATIC "${CMAKE_CURRENT_LIST_DIR}/VehicleSourcePlan.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PlanLimits.cpp" "${CMAKE_CURRENT_LIST_DIR}/PlanDeclarations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/PlanGeometry.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceCards.cpp")
target_link_libraries(robo_dyna_vehicle_source PUBLIC robo_dyna_source_assembly robo_dyna_full_shell_source_mapping)
target_compile_features(robo_dyna_vehicle_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_source PRIVATE -fno-fast-math -ffp-contract=off)
