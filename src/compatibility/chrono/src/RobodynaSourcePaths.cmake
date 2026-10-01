# Source relocation support for the retained CMake reference build.
# Bazel's foreign_cc bridge supplies an explicit root with declared build_data.
if(NOT DEFINED ROBODYNA_SOURCE_ROOT)
  get_filename_component(ROBODYNA_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
endif()
foreach(required IN ITEMS
    src/mechanics/inertia/RbMassProperties.cpp
    include/robodyna/mechanics/RbMassProperties.h)
  if(NOT EXISTS "${ROBODYNA_SOURCE_ROOT}/${required}")
    message(FATAL_ERROR "Missing owned Robodyna source: ${ROBODYNA_SOURCE_ROOT}/${required}")
  endif()
endforeach()
