include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/FixedContactFacetBinding.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/SurfaceMaterialMeasure.cmake")
find_package(Threads REQUIRED)
add_library(tl_represented_interval_crossing STATIC
  "${CMAKE_CURRENT_LIST_DIR}/RepresentedIntervalCrossing.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/RootIntervalFilter.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/RootIntervalFilter.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/RootIntervalQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/Batch.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/NormalReuseQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/ExactProjectionDomain.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/RelativeSeparationQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/BatchExecution.h")
target_link_libraries(tl_represented_interval_crossing
  PUBLIC tl_fixed_contact_facets tl_surface_material_measure Threads::Threads)
target_compile_features(tl_represented_interval_crossing PUBLIC cxx_std_17)
target_compile_options(tl_represented_interval_crossing
  PRIVATE -fno-fast-math -ffp-contract=off -frounding-math)
