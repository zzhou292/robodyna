include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/../../../vehicle_startup/shell_execution/tests/self_contact/CandidateFailureSupport.cmake")
# Internal diagnostic bridge; no public callback or additional mechanics owner.
add_library(robo_dyna_vehicle_run_observed_qualification STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Contribution.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/NativeSeal.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/RunAccess.cpp")
target_link_libraries(robo_dyna_vehicle_run_observed_qualification PUBLIC
  robo_dyna_vehicle_run robo_dyna_candidate_failure_fixture)
target_compile_features(robo_dyna_vehicle_run_observed_qualification PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_run_observed_qualification PRIVATE
  -fno-fast-math -ffp-contract=off)
