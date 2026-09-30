include_guard(GLOBAL)
# Intrinsic reference measure has no wall, mass, CUDA state or owner dependency.
add_library(tl_surface_material_measure STATIC
  "${CMAKE_CURRENT_LIST_DIR}/SurfaceMaterialMeasure.cpp")
get_filename_component(TL_SURFACE_MEASURE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(tl_surface_material_measure PUBLIC "${TL_SURFACE_MEASURE_ROOT}")
target_compile_features(tl_surface_material_measure PUBLIC cxx_std_17)
target_compile_options(tl_surface_material_measure PRIVATE -fno-fast-math -ffp-contract=off)
