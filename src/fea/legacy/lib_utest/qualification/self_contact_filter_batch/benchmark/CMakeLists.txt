cmake_minimum_required(VERSION 3.22)
project(TLSelfContactFilterBatchBenchmark LANGUAGES CXX CUDA)
get_filename_component(DEFAULT_TL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(TL_FILTER_ROOT "${DEFAULT_TL_ROOT}" CACHE PATH "Owning TL source for CPU/GPU certificate comparison")
set(TL_FILTER_FIXTURE_ROOT "" CACHE PATH "Optional explicit no-GTest native/Yaris fixture source root")
if(NOT EXISTS "${TL_FILTER_ROOT}/lib_src/collision/SelfContactFilterBatch.cmake")
  message(FATAL_ERROR "TL_FILTER_ROOT must contain the standalone filter batch")
endif()
include("${TL_FILTER_ROOT}/lib_src/collision/SelfContactFilterBatch.cmake")
add_executable(self_contact_filter_batch_benchmark Main.cpp Cases.cpp Results.cpp)
target_link_libraries(self_contact_filter_batch_benchmark PRIVATE tl_self_contact_filter_batch)
target_compile_features(self_contact_filter_batch_benchmark PRIVATE cxx_std_17)
target_compile_options(self_contact_filter_batch_benchmark PRIVATE -fno-fast-math -ffp-contract=off)
if(TL_FILTER_FIXTURE_ROOT)
  set(_fixture_dir "${TL_FILTER_FIXTURE_ROOT}/lib_utest/qualification/represented_interval_crossing")
  foreach(_file YarisGeometry.h PathFixture.h)
    if(NOT EXISTS "${_fixture_dir}/${_file}")
      message(FATAL_ERROR "Explicit fixture root must contain both no-GTest geometry helpers")
    endif()
  endforeach()
  find_package(Git REQUIRED)
  execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${TL_FILTER_FIXTURE_ROOT}" rev-parse HEAD
    OUTPUT_VARIABLE _fixture_commit OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _git_status)
  if(NOT _git_status EQUAL 0)
    message(FATAL_ERROR "Fixture source commit could not be pinned")
  endif()
  # Refresh the emitted metadata when a fixture header changes before rebuild.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${_fixture_dir}/YarisGeometry.h" "${_fixture_dir}/PathFixture.h")
  file(SHA256 "${_fixture_dir}/YarisGeometry.h" _geometry_sha)
  file(SHA256 "${_fixture_dir}/PathFixture.h" _path_sha)
  target_include_directories(self_contact_filter_batch_benchmark PRIVATE "${_fixture_dir}")
  target_compile_definitions(self_contact_filter_batch_benchmark PRIVATE
    TL_FILTER_YARIS_FIXTURES=1
    TL_FILTER_YARIS_COMMIT="${_fixture_commit}"
    TL_FILTER_YARIS_GEOMETRY_SHA256="${_geometry_sha}"
    TL_FILTER_YARIS_PATH_SHA256="${_path_sha}")
endif()
