include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../type25/VehicleType25Source.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../type45/VehicleType45Source.cmake")
add_library(robo_dyna_native_spring_ids STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ImportContext.cpp" "${CMAKE_CURRENT_LIST_DIR}/ContextSources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Elements.cpp" "${CMAKE_CURRENT_LIST_DIR}/Resolve.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ResolveValues.cpp")
target_link_libraries(robo_dyna_native_spring_ids PUBLIC robo_dyna_vehicle_type25_source robo_dyna_vehicle_type45_source)
target_compile_features(robo_dyna_native_spring_ids PUBLIC cxx_std_17)
