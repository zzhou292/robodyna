# Qualification-only composition. Native reference routines remain outside the
# production CUDA runtime; default builds do not enable this experiment.
add_library(qeph_wall_response_observer STATIC
  WallResponseModel.cpp WallResponseObservation.cpp WallResponseAnalytic.cpp WallResponseEvents.cpp)
target_link_libraries(qeph_wall_response_observer PUBLIC
  qeph_wall_recurrence_analysis qeph_response_observer)
add_library(qeph_wall_response_comparison STATIC
  WallResponseComparison.cpp WallResponseRunValidation.cpp)
target_link_libraries(qeph_wall_response_comparison PUBLIC qeph_wall_response_observer)
add_library(qeph_wall_response_report STATIC
  WallResponseSampleJson.cpp WallResponseSummaryJson.cpp WallResponseReport.cpp WallResponseRead.cpp)
target_link_libraries(qeph_wall_response_report PUBLIC
  qeph_wall_response_comparison qeph_response_report qeph_wall_raw_read)
foreach(target qeph_wall_response_observer qeph_wall_response_comparison qeph_wall_response_report)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -fno-fast-math -ffp-contract=off)
endforeach()

add_library(qeph_wall_response_runtime STATIC
  WallResponseInitialize.cu WallResponseTransaction.cu WallResponseRuntime.cu
  ../wall_coupled/WallCoupledFixture.cu ../wall_coupled/WallCoupledTransaction.cu
  ../wall_coupled/WallCoupledLedger.cu ../wall_coupled/ScreenedWallFixture.cu
  ../wall_coupled/ScreenedWallChecks.cu)
target_link_libraries(qeph_wall_response_runtime PUBLIC
  qeph_wall_response_observer qeph_coupled_fixture tl_nodal_wall_contact_device)
add_executable(qeph_wall_response_runtime_check
  WallResponseRuntimeTest.cu ../wall_incoming/WallIncomingFixture.cu)
target_link_libraries(qeph_wall_response_runtime_check PRIVATE qeph_wall_response_runtime GTest::gtest_main)
foreach(target qeph_wall_response_runtime qeph_wall_response_runtime_check)
  set_target_properties(${target} PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
  target_compile_options(${target} PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
    "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
endforeach()

add_executable(qeph_wall_response_observer_check WallResponseObserverTest.cpp)
target_link_libraries(qeph_wall_response_observer_check PRIVATE qeph_wall_response_observer GTest::gtest_main)
add_executable(qeph_wall_response_comparison_check WallResponseComparisonTest.cpp)
target_link_libraries(qeph_wall_response_comparison_check PRIVATE qeph_wall_response_comparison GTest::gtest_main)
add_executable(qeph_wall_response_report_check WallResponseReportTest.cpp)
target_link_libraries(qeph_wall_response_report_check PRIVATE qeph_wall_response_report GTest::gtest_main)
foreach(target qeph_wall_response_observer_check qeph_wall_response_comparison_check qeph_wall_response_report_check)
  target_compile_options(${target} PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME ${target} COMMAND ${target})
  set_tests_properties(${target} PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)
endforeach()
add_test(NAME qeph_wall_response_runtime_check COMMAND qeph_wall_response_runtime_check)
set_tests_properties(qeph_wall_response_runtime_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120)

# Full horizon is deliberately absent from default CTest: root launches one
# original-wall fixture/refinement per monitored process, coarse first.
add_executable(qeph_wall_response_run wall_response_run_main.cpp)
target_link_libraries(qeph_wall_response_run PRIVATE qeph_wall_response_runtime qeph_wall_response_report GTest::gtest)
add_executable(qeph_wall_response_compare wall_response_compare_main.cpp)
target_link_libraries(qeph_wall_response_compare PRIVATE qeph_wall_response_report)
foreach(target qeph_wall_response_run qeph_wall_response_compare)
  target_compile_options(${target} PRIVATE -fno-fast-math -ffp-contract=off)
endforeach()
