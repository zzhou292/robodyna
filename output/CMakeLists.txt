find_package(OpenSSL REQUIRED COMPONENTS Crypto)
add_library(robo_dyna_artifact_io STATIC ArtifactIO.cpp ArtifactInventory.cpp MeshArchive.cpp)
target_link_libraries(robo_dyna_artifact_io PUBLIC Chrono::Chrono_core PRIVATE OpenSSL::Crypto)
target_include_directories(robo_dyna_artifact_io PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_compile_features(robo_dyna_artifact_io PUBLIC cxx_std_17)

include("${CMAKE_CURRENT_LIST_DIR}/../case/CanonicalWall.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../case/CanonicalWallArtifacts.cmake")
add_library(robo_dyna_accepted_replay STATIC AcceptedReplay.cpp AcceptedReplayIndex.cpp AcceptedReplayMesh.cpp AcceptedReplayGuided.cpp)
target_link_libraries(robo_dyna_accepted_replay PUBLIC robo_dyna_artifact_io PRIVATE robo_dyna_canonical_wall_artifacts)
if(ROBO_DYNA_ENABLE_REPLAY_CHECKS OR ROBO_DYNA_ENABLE_ELASTIC_COUPON)
  find_package(GTest REQUIRED)
  add_executable(robo_dyna_accepted_replay_check accepted_replay_check.cpp)
  target_link_libraries(robo_dyna_accepted_replay_check PRIVATE robo_dyna_accepted_replay GTest::gtest_main)
  add_test(NAME accepted_replay COMMAND robo_dyna_accepted_replay_check
    --gtest_filter=-AcceptedReplay.ExplicitRetainedBundlePassesTheSameValidation)
  set_tests_properties(accepted_replay PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1)
  set(CRASH_CANONICAL_WALL "" CACHE FILEPATH "Prepared canonical Yaris wall manifest")
  if(CRASH_CANONICAL_WALL)
    if(NOT EXISTS "${CRASH_CANONICAL_WALL}")
      message(FATAL_ERROR "Declared canonical wall for guided replay checks must exist")
    endif()
    add_executable(robo_dyna_accepted_replay_guided_check accepted_replay_guided_check.cpp)
    target_link_libraries(robo_dyna_accepted_replay_guided_check PRIVATE
      robo_dyna_accepted_replay robo_dyna_canonical_wall_artifacts GTest::gtest_main)
    add_test(NAME accepted_replay_guided COMMAND robo_dyna_accepted_replay_guided_check
      --gtest_filter=-AcceptedReplayGuided.ExplicitRetainedGuidedBundleUsesTheSameStrictValidation)
    set_tests_properties(accepted_replay_guided PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
      ENVIRONMENT "ROBO_DYNA_CANONICAL_WALL=${CRASH_CANONICAL_WALL}")
  endif()
  foreach(kind NORMAL COUPON)
    set(ROBO_DYNA_REPLAY_FIXTURE_${kind} "" CACHE PATH "Optional retained ${kind} accepted-result bundle")
    if(ROBO_DYNA_REPLAY_FIXTURE_${kind})
      if(NOT EXISTS "${ROBO_DYNA_REPLAY_FIXTURE_${kind}}/manifest.json")
        message(FATAL_ERROR "Declared ${kind} replay fixture must have a completed manifest")
      endif()
      add_test(NAME accepted_replay_${kind} COMMAND robo_dyna_accepted_replay_check
        --gtest_filter=AcceptedReplay.ExplicitRetainedBundlePassesTheSameValidation)
      set_tests_properties(accepted_replay_${kind} PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
        ENVIRONMENT "ROBO_DYNA_REPLAY_FIXTURE=${ROBO_DYNA_REPLAY_FIXTURE_${kind}}")
    endif()
  endforeach()
  set(ROBO_DYNA_GUIDED_REPLAY_FIXTURE "" CACHE PATH "Optional retained guided accepted-result bundle")
  if(ROBO_DYNA_GUIDED_REPLAY_FIXTURE)
    if(NOT TARGET robo_dyna_accepted_replay_guided_check OR
       NOT EXISTS "${ROBO_DYNA_GUIDED_REPLAY_FIXTURE}/manifest.json")
      message(FATAL_ERROR "Declared guided replay fixture requires the canonical wall and completed manifest")
    endif()
    add_test(NAME accepted_replay_guided_retained COMMAND robo_dyna_accepted_replay_guided_check
      --gtest_filter=AcceptedReplayGuided.ExplicitRetainedGuidedBundleUsesTheSameStrictValidation)
    set_tests_properties(accepted_replay_guided_retained PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE PROCESSORS 1
      ENVIRONMENT "ROBO_DYNA_GUIDED_REPLAY_FIXTURE=${ROBO_DYNA_GUIDED_REPLAY_FIXTURE}")
  endif()
endif()
