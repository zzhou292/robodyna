include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/SourceAssemblyWallValues.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../case/source_assembly_observation/SourceAssemblyObservation.cmake")
add_library(robo_dyna_source_assembly_spin_fields STATIC
  "${CMAKE_CURRENT_LIST_DIR}/QephSpinFields.cpp" "${CMAKE_CURRENT_LIST_DIR}/QephSpinPlan.cpp")
target_link_libraries(robo_dyna_source_assembly_spin_fields PUBLIC robo_dyna_source_assembly_wall_values robo_dyna_source_assembly_observation)
target_compile_features(robo_dyna_source_assembly_spin_fields PUBLIC cxx_std_17)
target_compile_options(robo_dyna_source_assembly_spin_fields PRIVATE -fno-fast-math -ffp-contract=off)
