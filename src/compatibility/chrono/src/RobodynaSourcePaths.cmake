# Source relocation support for the retained CMake reference build.
# Bazel's foreign_cc bridge supplies an explicit root with declared build_data.
if(NOT DEFINED ROBODYNA_SOURCE_ROOT)
  get_filename_component(ROBODYNA_SOURCE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
endif()
foreach(required IN ITEMS
    include/robodyna/core/RbVector3.h
    src/simulation/composition/RbSystem.cpp
    include/robodyna/simulation/RbSystem.h
    src/simulation/composition/RbSystemNSC.cpp
    include/robodyna/simulation/RbSystemNSC.h
    src/simulation/composition/RbSystemSMC.cpp
    include/robodyna/simulation/RbSystemSMC.h
    include/robodyna/simulation/RbSystemFwd.h
    src/mbd/bodies/RbBodyAuxRef.cpp
    src/mbd/bodies/RbBodyEasy.cpp
    include/robodyna/mbd/RbBodyAuxRef.h
    include/robodyna/mbd/RbBodyEasy.h
    src/mbd/bodies/RbBody.cpp
    include/robodyna/mbd/RbBody.h
    include/robodyna/mbd/RbBodyFwd.h
    src/mechanics/inertia/RbMassProperties.cpp
    include/robodyna/mechanics/RbMassProperties.h)
  if(NOT EXISTS "${ROBODYNA_SOURCE_ROOT}/${required}")
    message(FATAL_ERROR "Missing owned Robodyna source: ${ROBODYNA_SOURCE_ROOT}/${required}")
  endif()
endforeach()
