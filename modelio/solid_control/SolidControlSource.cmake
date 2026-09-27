include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../native_spring_ids/NativeSpringIds.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../self_contact/OriginalSelection.cmake")
add_library(robo_dyna_solid_control_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/DirectSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/DirectDeclarations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/EffectiveContext.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/EffectiveSource.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Sharing.cpp")
target_link_libraries(robo_dyna_solid_control_source PUBLIC
  robo_dyna_native_spring_ids robo_dyna_original_self_contact_selection)
target_compile_features(robo_dyna_solid_control_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_solid_control_source PRIVATE -fno-fast-math -ffp-contract=off)
