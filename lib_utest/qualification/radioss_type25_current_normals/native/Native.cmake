include_guard(GLOBAL)
# Add the owning fixed-main startup project first; reuse its one complete native
# NORMP/FREE_BOUND target and its private COMMON/modules/precision environment.
if(NOT TARGET type25_startup_oracle)
  message(FATAL_ERROR "Current-normal oracle requires existing complete startup oracle")
endif()
get_filename_component(current_normal_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
target_sources(type25_startup_oracle PRIVATE "${CMAKE_CURRENT_LIST_DIR}/CurrentWrapper.F90")
add_library(type25_current_normals_oracle STATIC "${current_normal_root}/NativeOracle.cpp")
target_link_libraries(type25_current_normals_oracle PUBLIC type25_startup_oracle)
target_compile_features(type25_current_normals_oracle PUBLIC cxx_std_17)
target_compile_options(type25_current_normals_oracle PRIVATE -fno-fast-math -ffp-contract=off)
