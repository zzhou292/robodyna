# Owning build adapter for the unchanged DEME source. All additional headers and
# NVRTC come from declared foreign_cc dependencies, not a host math installation.
if(NOT DEFINED ROBODYNA_DEME_SDK_PREFIX)
  message(FATAL_ERROR "Declare ROBODYNA_DEME_SDK_PREFIX from the foreign build inputs")
endif()
foreach(header jitify.hpp nvrtc.h cub/cub.cuh)
  if(NOT EXISTS "${ROBODYNA_DEME_SDK_PREFIX}/include/${header}")
    message(FATAL_ERROR "Missing declared DEME SDK header: ${header}")
  endif()
endforeach()
# foreign_cc stages SDK headers as symlinks into its temporary dependency tree.
# The inherited install(FILES) preserves that symlink, leaving an invalid output
# after sandbox cleanup. Materialize only this declared header before the normal
# source build/install; configure_file(COPYONLY) follows its input symlink.
set(_robodyna_jitify_directory "${CMAKE_BINARY_DIR}/robodyna_sdk/include")
file(MAKE_DIRECTORY "${_robodyna_jitify_directory}")
configure_file("${ROBODYNA_DEME_SDK_PREFIX}/include/jitify.hpp"
               "${_robodyna_jitify_directory}/jitify.hpp" COPYONLY)
file(SHA256 "${ROBODYNA_DEME_SDK_PREFIX}/include/jitify.hpp" _robodyna_jitify_source_sha)
file(SHA256 "${_robodyna_jitify_directory}/jitify.hpp" _robodyna_jitify_copy_sha)
if(IS_SYMLINK "${_robodyna_jitify_directory}/jitify.hpp" OR
   NOT _robodyna_jitify_source_sha STREQUAL _robodyna_jitify_copy_sha)
  message(FATAL_ERROR "The declared Jitify header was not materialized byte for byte")
endif()
set(NVIDIAJitifyPath "${_robodyna_jitify_directory}" CACHE PATH "Declared retained Jitify header" FORCE)
set(CUDA_nvrtc_LIBRARY "${ROBODYNA_DEME_SDK_PREFIX}/lib/libnvrtc.so.13" CACHE FILEPATH "Declared pinned NVRTC" FORCE)
if(NOT EXISTS "${CUDA_nvrtc_LIBRARY}")
  message(FATAL_ERROR "Missing declared NVRTC shared library")
endif()
include_directories(SYSTEM "${ROBODYNA_DEME_SDK_PREFIX}/include")

# CUDA13 locates CCCL beneath include/cccl. Keep its actual host-toolkit path in
# DEME's existing runtime-JIT include metadata; never bake the temporary staged
# dependency directory into that runtime field.
set(_robodyna_cuda_includes "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")
foreach(directory IN LISTS _robodyna_cuda_includes)
  if(EXISTS "${directory}/cccl/cub/cub.cuh")
    list(APPEND CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES "${directory}/cccl")
  endif()
endforeach()
list(REMOVE_DUPLICATES CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES)

# The retained core target passes this list through a command-line definition.
# With two include roots, CMake splits its semicolon into separate shell tokens.
# Put exactly the same runtime string in a generated header instead. The owning
# adapter executes after add_subdirectory(src) has created the core target.
set_property(GLOBAL PROPERTY ROBODYNA_DEME_RUNTIME_INCLUDE_DIRS
             "${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}")
function(robodyna_finalize_deme_runtime_includes)
  if(NOT TARGET core)
    message(FATAL_ERROR "Retained DEME core target is missing")
  endif()
  get_target_property(definitions core COMPILE_DEFINITIONS)
  # The inherited generator expression itself may already have been split by
  # its unescaped list. It is the sole compile definition on this pinned target.
  get_property(runtime_includes GLOBAL PROPERTY ROBODYNA_DEME_RUNTIME_INCLUDE_DIRS)
  set(expected "DEME_CUDA_TOOLKIT_INCLUDE_DIRS=\"$<JOIN:${runtime_includes},;>\"")
  if(NOT "${definitions}" STREQUAL "${expected}")
    message(FATAL_ERROR "Review changed DEME core compile definitions before adapting them")
  endif()
  set_property(TARGET core PROPERTY COMPILE_DEFINITIONS "")
  string(REPLACE "\\" "\\\\" runtime_includes "${runtime_includes}")
  string(REPLACE "\"" "\\\"" runtime_includes "${runtime_includes}")
  set(header "${CMAKE_BINARY_DIR}/robodyna_deme_runtime_includes.h")
  file(WRITE "${header}" "#pragma once\n#define DEME_CUDA_TOOLKIT_INCLUDE_DIRS \"${runtime_includes}\"\n")
  target_compile_options(core PRIVATE -include "${header}")

  # All source install rules now exist. Copy their declared header/kernel/data
  # symlink inputs before foreign_cc removes the action's temporary source tree.
  install(CODE "set(ROBODYNA_DEME_INSTALL_INCLUDE_DIR \"${CMAKE_INSTALL_INCLUDEDIR}\")\nset(ROBODYNA_DEME_INSTALL_DATA_DIR \"${CMAKE_INSTALL_DATADIR}\")\nset(ROBODYNA_DEME_JITIFY_SHA256 \"${_robodyna_jitify_source_sha}\")")
  install(SCRIPT "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/deme_materialize_install.cmake")
endfunction()
cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}"
               CALL robodyna_finalize_deme_runtime_includes)
