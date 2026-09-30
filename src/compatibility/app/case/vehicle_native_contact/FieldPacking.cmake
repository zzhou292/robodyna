include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../output/ArtifactIO.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/RadiossType25FixedMainStartup.cmake")
add_library(robo_dyna_native_vehicle_field_packing STATIC "${CMAKE_CURRENT_LIST_DIR}/FieldPacking.cpp")
target_link_libraries(robo_dyna_native_vehicle_field_packing PUBLIC
  robo_dyna_artifact_io tl_radioss_type25_fixed_main_startup)
target_compile_features(robo_dyna_native_vehicle_field_packing PUBLIC cxx_std_17)
target_compile_options(robo_dyna_native_vehicle_field_packing PRIVATE -fno-fast-math -ffp-contract=off)
