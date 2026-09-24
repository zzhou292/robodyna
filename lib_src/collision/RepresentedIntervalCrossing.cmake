include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/FixedContactFacetBinding.cmake")
find_package(Threads REQUIRED)
add_library(tl_represented_interval_crossing STATIC
  "${CMAKE_CURRENT_LIST_DIR}/RepresentedIntervalCrossing.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/DeviceExecution.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/CanonicalPair.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/CohortAdmission.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/BusyRelease.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/Batch.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/NormalReuseQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/ExactPathReuseQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/CommonPointReuseQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/NativeStorageQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/FixedPolicyQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/ArithmeticContext.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/BoostIntegerPolicy.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/FixedIntegerPolicy.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/NativeStorageDomain.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/Modes.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/PortableStd.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/Identity.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/Arithmetic.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/Geometry.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/native/CellKernel.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/ExactProjectionDomain.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/RelativeSeparationQualification.h"
  "${CMAKE_CURRENT_LIST_DIR}/represented_interval_crossing/BatchExecution.h")
target_link_libraries(tl_represented_interval_crossing
  PUBLIC tl_fixed_contact_facets Threads::Threads)
target_compile_features(tl_represented_interval_crossing PUBLIC cxx_std_17)
target_compile_options(tl_represented_interval_crossing
  PRIVATE -fno-fast-math -ffp-contract=off)
