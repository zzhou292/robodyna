include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../RigidPartSource.cmake")
add_library(robo_dyna_rigid_point_mass_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Fields.cpp" "${CMAKE_CURRENT_LIST_DIR}/Source.cpp")
target_link_libraries(robo_dyna_rigid_point_mass_source PUBLIC robo_dyna_rigid_part_source)
target_compile_features(robo_dyna_rigid_point_mass_source PUBLIC cxx_std_17)
target_compile_options(robo_dyna_rigid_point_mass_source PRIVATE -fno-fast-math -ffp-contract=off)
