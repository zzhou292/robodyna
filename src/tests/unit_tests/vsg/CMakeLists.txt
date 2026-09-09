set(TESTS utest_VSG_LoadingThreads)
set(LIBS Chrono_vsg Chrono_core)
build_utests(${BUILD_TESTING_VSG} "${TESTS}" "vsg" "${LIBS}")
set_tests_properties(utest_VSG_LoadingThreads PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
