include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../vehicle_source/VehicleSourcePlan.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../rigid_part/RigidPartSource.cmake")
add_library(robo_dyna_vehicle_sections STATIC
  "${CMAKE_CURRENT_LIST_DIR}/VehicleSectionResolution.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ResolutionDeclarations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ResolutionParents.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ResolutionLimits.cpp")
target_sources(robo_dyna_vehicle_sections PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/GlassDeclarations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/GlassSourceCards.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/GlassPolicy.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MidlayerDeclarations.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MidlayerProfile.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RigidProfile.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NativeParentMapping.cpp")
target_link_libraries(robo_dyna_vehicle_sections PUBLIC robo_dyna_vehicle_source robo_dyna_rigid_part_source)
target_compile_features(robo_dyna_vehicle_sections PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_sections PRIVATE -fno-fast-math -ffp-contract=off)
