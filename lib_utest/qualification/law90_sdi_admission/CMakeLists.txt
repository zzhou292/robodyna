cmake_minimum_required(VERSION 3.22)
project(law90_sdi_admission LANGUAGES C CXX Fortran)
enable_testing()
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Boost 1.74 EXACT REQUIRED)
# The installed runtime and matching headers exist; only its development CMake
# component/symlink is absent. Use the real versioned library without installing
# packages or creating system links.
find_library(TL_LAW90_BOOST_FILESYSTEM NAMES boost_filesystem libboost_filesystem.so.1.74.0 REQUIRED)
add_library(law90_sdi_boost_filesystem SHARED IMPORTED)
set_target_properties(law90_sdi_boost_filesystem PROPERTIES
  IMPORTED_LOCATION "${TL_LAW90_BOOST_FILESYSTEM}" INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
file(SHA256 "${TL_LAW90_BOOST_FILESYSTEM}" boost_filesystem_sha)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/boost-dependency.txt"
  "headers=${Boost_INCLUDE_DIRS}\nlibrary=${TL_LAW90_BOOST_FILESYSTEM}\nsha256=${boost_filesystem_sha}\n")
set(TL_LAW90_SDI_CACHE "" CACHE PATH "Authenticated openradioss-law90-sdi-1 cache")
set(TL_LAW90_SDI_UNITS "" CACHE FILEPATH "Authenticated complete hm_get_floatv.F")
set(TL_LAW90_RADIATOR_FIXTURE "" CACHE PATH "Authenticated original geometry receipt directory")
set(pinned "${CMAKE_CURRENT_BINARY_DIR}/pinned")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  --cache "${TL_LAW90_SDI_CACHE}" --units "${TL_LAW90_SDI_UNITS}" --output "${pinned}"
  COMMAND_ERROR_IS_FATAL ANY)
# Reuse all five complete native core-library CMake definitions. The solver's
# unrelated Fortran bridge is not built. The native writer needs only its
# documented current-model storage accessor, supplied in Writer.cpp.
set(arch linux64)
set(com 0)
set(lib a)
set(libp lib)
set(open_reader_root_dir "${pinned}/reader")
set(build_dir cbuild_qualification)
file(GLOB_RECURSE headers "${open_reader_root_dir}/source/*.h")
set(include_dir_list "")
foreach(header IN LISTS headers)
  get_filename_component(dir "${header}" DIRECTORY)
  list(APPEND include_dir_list "${dir}")
endforeach()
list(REMOVE_DUPLICATES include_dir_list)
include("${open_reader_root_dir}/CMake_arch/CMake_includes.cmake")
foreach(module IN ITEMS dyna2rad cfgkernel io sdi cfgio)
  add_subdirectory("${open_reader_root_dir}/source/${module}" "reader-${module}")
endforeach()
add_library(law90_sdi_reader INTERFACE)
target_include_directories(law90_sdi_reader INTERFACE ${include_dir_list} ${Boost_INCLUDE_DIRS})
target_link_libraries(law90_sdi_reader INTERFACE
  -Wl,--start-group dyna2rad_linux64 io_linux64 sdi_linux64 cfgio_linux64 cfgkernel_linux64
  -Wl,--end-group law90_sdi_boost_filesystem)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../law90_preparation/native" preparation)
add_executable(law90_sdi_native Main.cpp Fields.cpp Writer.cpp Preparation.cpp NativeUnits.F
  "${open_reader_root_dir}/source/solver_interface/source/cfg_reading/PrintOption.cpp")
target_compile_features(law90_sdi_native PRIVATE cxx_std_17)
target_include_directories(law90_sdi_native PRIVATE "${pinned}")
target_compile_definitions(law90_sdi_native PRIVATE NO_GZ_LIB TXT_MSG UNV USE_NAMESPACE
  BOOST_ALL_NO_LIB NDEBUG OS_UNIX LINUX LINUX_PLATFORM)
target_compile_options(law90_sdi_native PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-fpermissive;-fno-strict-aliasing>
  $<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-fcheck=bounds;-fno-fast-math;-ffp-contract=off>)
target_link_libraries(law90_sdi_native PRIVATE law90_sdi_reader law90_preparation_native)
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/fixture.py"
  --receipt "${TL_LAW90_RADIATOR_FIXTURE}/manifest.json" --output "${CMAKE_CURRENT_BINARY_DIR}/fixture"
  COMMAND_ERROR_IS_FATAL ANY)
add_test(NAME law90_sdi_source_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py" --cache "${TL_LAW90_SDI_CACHE}"
  --units "${TL_LAW90_SDI_UNITS}" --check)
add_test(NAME law90_sdi_direct_export_read COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_SOURCE_DIR}/run_check.py" --executable "$<TARGET_FILE:law90_sdi_native>"
  --pinned "${pinned}" --fixture "${CMAKE_CURRENT_BINARY_DIR}/fixture"
  --output "${CMAKE_CURRENT_BINARY_DIR}/observations")
