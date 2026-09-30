include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../source_assembly/Reader.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/type13/Type13Startup.cmake")
if(NOT TARGET robo_dyna_bounded_array_io)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../../output/arrays" "${CMAKE_CURRENT_BINARY_DIR}/type13_arrays")
endif()
add_library(robo_dyna_type13_source STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ConvertProperty.cpp" "${CMAKE_CURRENT_LIST_DIR}/SourceType13.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadScope.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReadProperty.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ReadGeometry.cpp" "${CMAKE_CURRENT_LIST_DIR}/ReadSourceArrays.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/SourceBudget.cpp")
target_link_libraries(robo_dyna_type13_source PUBLIC robo_dyna_source_assembly tl_type13_startup robo_dyna_bounded_array_io)
target_compile_options(robo_dyna_type13_source PRIVATE -fno-fast-math -ffp-contract=off)
