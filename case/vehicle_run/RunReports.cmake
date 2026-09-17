include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/Values.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../output/physical_run/PhysicalRun.cmake")
add_library(robo_dyna_vehicle_run_reports STATIC
  "${CMAKE_CURRENT_LIST_DIR}/ContactSummary.cpp" "${CMAKE_CURRENT_LIST_DIR}/Summary.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/MechanicsSummary.cpp" "${CMAKE_CURRENT_LIST_DIR}/MechanicsDocument.cpp")
target_sources(robo_dyna_vehicle_run_reports PRIVATE "${CMAKE_CURRENT_LIST_DIR}/SampledShellPlasticity.cpp")
target_sources(robo_dyna_vehicle_run_reports PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/SelfContactSummary.cpp" "${CMAKE_CURRENT_LIST_DIR}/SelfContactDocument.cpp")
target_include_directories(robo_dyna_vehicle_run_reports PUBLIC "${ROBO_DYNA_TL_ROOT}" "${ROBO_DYNA_TL_ROOT}/lib_src" "${CUDAToolkit_INCLUDE_DIRS}")
target_link_libraries(robo_dyna_vehicle_run_reports PUBLIC robo_dyna_vehicle_run_values robo_dyna_physical_run_records)
target_compile_features(robo_dyna_vehicle_run_reports PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_run_reports PRIVATE -fno-fast-math -ffp-contract=off)
