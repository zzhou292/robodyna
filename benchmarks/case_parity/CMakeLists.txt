cmake_minimum_required(VERSION 3.22)
project(RoboDynaCaseParity LANGUAGES NONE)
enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(ROBO_APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_test(NAME case_parity_host
  COMMAND "${Python3_EXECUTABLE}" -B -m unittest
    benchmarks.case_parity.tests.test_contracts
    benchmarks.case_parity.tests.test_performance
    benchmarks.case_parity.tests.test_real_case
    viewer.postprocess.test_replay_evidence -v)
set_tests_properties(case_parity_host PROPERTIES
  WORKING_DIRECTORY "${ROBO_APP_ROOT}" RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60
  ENVIRONMENT "PYTHONPATH=${ROBO_APP_ROOT}")

add_test(NAME case_parity_existing_payload_reader
  COMMAND "${Python3_EXECUTABLE}" -B
    "${ROBO_APP_ROOT}/benchmarks/accepted_payloads/test_compare.py")
set_tests_properties(case_parity_existing_payload_reader PROPERTIES
  WORKING_DIRECTORY "${ROBO_APP_ROOT}" RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 60)
