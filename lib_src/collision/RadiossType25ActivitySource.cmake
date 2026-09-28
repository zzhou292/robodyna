include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/RadiossType25PhysicalMainSource.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../elements/type45/Model.cmake")
set(_type25_activity_source "${CMAKE_CURRENT_LIST_DIR}/radioss_type25/activity_source")
add_library(tl_radioss_type25_activity_source STATIC
  "${_type25_activity_source}/Emission.cpp" "${_type25_activity_source}/Parents.cpp" "${_type25_activity_source}/Mains.cpp" "${_type25_activity_source}/Plan.cpp")
target_link_libraries(tl_radioss_type25_activity_source PUBLIC tl_radioss_type25_physical_main_source tl_type45_model)
target_compile_features(tl_radioss_type25_activity_source PUBLIC cxx_std_17)
target_compile_options(tl_radioss_type25_activity_source PRIVATE -fno-fast-math -ffp-contract=off)
