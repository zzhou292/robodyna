if(NOT TARGET robo_dyna_elastic_shell_envelope)
  add_library(robo_dyna_elastic_shell_envelope STATIC ElasticShellEnvelope.cpp)
  target_compile_features(robo_dyna_elastic_shell_envelope PUBLIC cxx_std_17)
  target_include_directories(robo_dyna_elastic_shell_envelope PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CRASH_TL_FEA_SOURCE_DIR}")
  target_link_libraries(robo_dyna_elastic_shell_envelope PUBLIC tl_reissner_shell_batch)
  target_compile_options(robo_dyna_elastic_shell_envelope PRIVATE -fno-fast-math -ffp-contract=off)
endif()
