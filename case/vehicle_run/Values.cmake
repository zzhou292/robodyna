include_guard(GLOBAL)
get_filename_component(vehicle_run_app_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_library(robo_dyna_vehicle_run_values STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Config.cpp" "${CMAKE_CURRENT_LIST_DIR}/ContactProfile.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Loop.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/cli/Options.cpp")
target_include_directories(robo_dyna_vehicle_run_values PUBLIC "${vehicle_run_app_root}")
target_compile_features(robo_dyna_vehicle_run_values PUBLIC cxx_std_17)
target_compile_options(robo_dyna_vehicle_run_values PRIVATE -fno-fast-math -ffp-contract=off)
