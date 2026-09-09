# External-project linkage follows chrono/template_project/CMakeLists.txt.
enable_language(CXX)
find_package(Chrono REQUIRED CONFIG)

add_executable(crash_chrono_wall_check wall_check.cpp)
target_compile_features(crash_chrono_wall_check PRIVATE cxx_std_17)
target_link_libraries(crash_chrono_wall_check PRIVATE ${CHRONO_TARGETS})

set(CRASH_WALL_OBJ "" CACHE FILEPATH "Prepared pinned Yaris wall OBJ for the integration check")
if(NOT EXISTS "${CRASH_WALL_OBJ}")
  message(FATAL_ERROR "CRASH_WALL_OBJ must name the compiled Yaris wall.obj")
endif()
add_test(NAME chrono_wall_integration
  COMMAND crash_chrono_wall_check "${CRASH_WALL_OBJ}")
set_tests_properties(chrono_wall_integration PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
