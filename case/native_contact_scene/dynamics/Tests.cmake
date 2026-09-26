add_executable(native_scene_dynamics_host_check "${CMAKE_CURRENT_LIST_DIR}/tests/OptionsTest.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tests/ForecastTest.cpp")
target_link_libraries(native_scene_dynamics_host_check PRIVATE robo_dyna_native_scene_options GTest::gtest_main)
add_test(NAME native_scene_dynamics_host COMMAND native_scene_dynamics_host_check)
set_tests_properties(native_scene_dynamics_host PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120
  ENVIRONMENT "ROBO_DYNA_NATIVE_SCENE_EXPORT=${ROBO_DYNA_NATIVE_SCENE_EXPORT}")
add_executable(native_scene_dynamics_cuda_check "${CMAKE_CURRENT_LIST_DIR}/tests/CaptureTest.cpp")
target_link_libraries(native_scene_dynamics_cuda_check PRIVATE robo_dyna_native_scene_run GTest::gtest_main CUDA::cudart)
set(TYPE25_NATIVE_REFERENCE_FILE "" CACHE FILEPATH "Pinned independent native1000interval reference, qualification only")
if(TYPE25_NATIVE_REFERENCE_FILE)
  if(NOT EXISTS "${TYPE25_NATIVE_REFERENCE_FILE}" OR NOT EXISTS "${ROBO_DYNA_NATIVE_SCENE_EXPECTED_INCLUDE}/ObservedScene.h")
    message(FATAL_ERROR "Actual native trajectory gate requires both pinned reference files")
  endif()
  target_sources(native_scene_dynamics_cuda_check PRIVATE "${CMAKE_CURRENT_LIST_DIR}/tests/NativeTrajectoryTest.cpp")
  target_include_directories(native_scene_dynamics_cuda_check PRIVATE "${ROBO_DYNA_NATIVE_SCENE_EXPECTED_INCLUDE}")
  target_compile_definitions(native_scene_dynamics_cuda_check PRIVATE TYPE25_NATIVE_REFERENCE_FILE="${TYPE25_NATIVE_REFERENCE_FILE}")
endif()
add_test(NAME native_scene_dynamics_cuda COMMAND native_scene_dynamics_cuda_check)
set_tests_properties(native_scene_dynamics_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 2 TIMEOUT 600
  ENVIRONMENT "ROBO_DYNA_NATIVE_SCENE_EXPORT=${ROBO_DYNA_NATIVE_SCENE_EXPORT}")
foreach(t native_scene_dynamics_host_check native_scene_dynamics_cuda_check)
  target_compile_options(${t} PRIVATE -fno-fast-math -ffp-contract=off)
endforeach()

if(ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT)
  target_sources(native_scene_dynamics_host_check PRIVATE "${CMAKE_CURRENT_LIST_DIR}/tests/MovingForecastTest.cpp")
  set_property(TEST native_scene_dynamics_host APPEND PROPERTY ENVIRONMENT
    "ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT=${ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT}")
endif()
set(TYPE25_MOVING_NATIVE_REFERENCE_DIR "" CACHE PATH "Pinned expected v2 trajectory and metadata, qualification only")
if(TYPE25_MOVING_NATIVE_REFERENCE_DIR)
  set(moving_binary "${TYPE25_MOVING_NATIVE_REFERENCE_DIR}/native-reference.bin")
  set(moving_metadata "${TYPE25_MOVING_NATIVE_REFERENCE_DIR}/reference-metadata.json")
  if(NOT EXISTS "${moving_binary}" OR NOT EXISTS "${moving_metadata}" OR NOT ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT)
    message(FATAL_ERROR "Moving trajectory gate requires v2 source and both expected reference artifacts")
  endif()
  add_executable(native_scene_moving_dynamics_cuda_check "${CMAKE_CURRENT_LIST_DIR}/tests/MovingTrajectoryTest.cpp")
  target_link_libraries(native_scene_moving_dynamics_cuda_check PRIVATE robo_dyna_native_scene_run GTest::gtest_main CUDA::cudart)
  target_compile_definitions(native_scene_moving_dynamics_cuda_check PRIVATE
    TYPE25_MOVING_NATIVE_REFERENCE_FILE="${moving_binary}"
    TYPE25_MOVING_NATIVE_REFERENCE_METADATA="${moving_metadata}")
  target_compile_options(native_scene_moving_dynamics_cuda_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME native_scene_moving_dynamics_cuda COMMAND native_scene_moving_dynamics_cuda_check)
  set_tests_properties(native_scene_moving_dynamics_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 2 TIMEOUT 600
    ENVIRONMENT "ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT=${ROBO_DYNA_NATIVE_MOVING_SCENE_EXPORT}")
endif()


set(TYPE25_RIGID_NATIVE_REFERENCE_DIR "" CACHE PATH "Pinned expected v3 contact/rigid phases, qualification only")
if(TYPE25_RIGID_NATIVE_REFERENCE_DIR)
  target_sources(native_scene_dynamics_host_check PRIVATE "${CMAKE_CURRENT_LIST_DIR}/tests/RigidGaugeTest.cpp")
  set(rigid_contact "${TYPE25_RIGID_NATIVE_REFERENCE_DIR}/native-reference.bin")
  set(rigid_groups "${TYPE25_RIGID_NATIVE_REFERENCE_DIR}/rigid-reference.bin")
  set(rigid_metadata "${TYPE25_RIGID_NATIVE_REFERENCE_DIR}/reference-metadata.json")
  if(NOT EXISTS "${rigid_contact}" OR NOT EXISTS "${rigid_groups}" OR NOT EXISTS "${rigid_metadata}" OR NOT ROBO_DYNA_NATIVE_RIGID_SCENE_EXPORT)
    message(FATAL_ERROR "Rigid trajectory gate requires v3 source and all independently converted expected artifacts")
  endif()
  add_executable(native_scene_rigid_dynamics_cuda_check "${CMAKE_CURRENT_LIST_DIR}/tests/RigidTrajectoryTest.cpp")
  target_link_libraries(native_scene_rigid_dynamics_cuda_check PRIVATE robo_dyna_native_scene_run GTest::gtest_main CUDA::cudart)
  target_compile_definitions(native_scene_rigid_dynamics_cuda_check PRIVATE
    TYPE25_RIGID_NATIVE_REFERENCE_FILE="${rigid_contact}" TYPE25_RIGID_NATIVE_GROUP_FILE="${rigid_groups}"
    TYPE25_RIGID_NATIVE_REFERENCE_METADATA="${rigid_metadata}")
  target_compile_options(native_scene_rigid_dynamics_cuda_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME native_scene_rigid_dynamics_cuda COMMAND native_scene_rigid_dynamics_cuda_check)
  set_tests_properties(native_scene_rigid_dynamics_cuda PROPERTIES RUN_SERIAL TRUE PROCESSORS 2 TIMEOUT 600
    ENVIRONMENT "ROBO_DYNA_NATIVE_RIGID_SCENE_EXPORT=${ROBO_DYNA_NATIVE_RIGID_SCENE_EXPORT}")
endif()
