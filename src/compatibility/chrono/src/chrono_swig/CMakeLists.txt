# Resolve the owned source root in this parent scope for both binding children.
include("${CMAKE_CURRENT_LIST_DIR}/../RobodynaSourcePaths.cmake")

add_subdirectory(chrono_python)
add_subdirectory(chrono_csharp)

if(NOT CH_ENABLE_MODULE_PYTHON AND NOT CH_ENABLE_MODULE_CSHARP)
    return()
endif()

# Optionally install SWIG interface files
option(CH_INSTALL_SWIG_INTERFACE "Install the Chrono SWIG interface files" OFF)

if(CH_INSTALL_SWIG_INTERFACE)
    message(STATUS "\n==== Chrono SWIG module ====\n")

    file(COPY ${CMAKE_SOURCE_DIR}/src/chrono_swig/interface/ DESTINATION ${CMAKE_BINARY_DIR}/chrono_swig)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_swig/interface/ DESTINATION include/chrono_swig)

    # Shipped interfaces include every enabled authenticated parser dependency.
    get_property(declaration_targets GLOBAL PROPERTY ROBODYNA_SWIG_DECLARATION_TARGETS)
    get_property(declaration_files GLOBAL PROPERTY ROBODYNA_SWIG_DECLARATION_FILES)
    if(NOT declaration_targets OR NOT declaration_files)
        message(FATAL_ERROR "SWIG interface installation requires generated declaration views")
    endif()
    add_custom_target(robodyna_swig_install_inputs ALL)
    add_dependencies(robodyna_swig_install_inputs ${declaration_targets})
    install(FILES ${declaration_files} DESTINATION include/robodyna_swig)

    message(STATUS "Chrono SWIG interface files copied to ${CMAKE_BINARY_DIR}/chrono_swig")
    message(STATUS "Chrono SWIG interface will be installed to ${CMAKE_INSTALL_PREFIX}/include/chrono_swig")
endif()
