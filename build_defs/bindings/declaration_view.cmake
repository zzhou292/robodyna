# Generated declarations are SWIG inputs only, never native class definitions.
include_guard(GLOBAL)

function(_robodyna_bind_swig_view target family ledger_relative canonical_relative forwarder_relative)
  set(view_target "robodyna_swig_${family}_declarations")
  set(contract "${ROBODYNA_SOURCE_ROOT}/build_defs/bindings/${family}_view_contract.json")
  # Reconfigure command arguments when the reviewed pins/output contract change.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${contract}")
  file(READ "${contract}" contract_json)
  string(JSON output_relative GET "${contract_json}" output)
  if(NOT output_relative MATCHES "^robodyna_swig/[A-Za-z0-9_]+[.]h$")
    message(FATAL_ERROR "Declaration output must stay in the owned robodyna_swig build directory")
  endif()
  set(view "${PROJECT_BINARY_DIR}/${output_relative}")
  if(NOT TARGET ${view_target})
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    string(JSON original_path GET "${contract_json}" original_path)
    string(JSON original_sha GET "${contract_json}" expected_original_sha256)
    string(JSON ledger_sha GET "${contract_json}" expected_ledger_sha256)
    set(ledger "${ROBODYNA_SOURCE_ROOT}/${ledger_relative}")
    set(generator "${ROBODYNA_SOURCE_ROOT}/tools/bindings/declaration_view.py")
    set(helper "${ROBODYNA_SOURCE_ROOT}/tools/migration/source_transform.py")
    get_filename_component(view_dir "${view}" DIRECTORY)
    file(MAKE_DIRECTORY "${view_dir}")
    add_custom_command(
      OUTPUT "${view}" "${view}.json"
      # Only these owned build outputs are replaced during incremental rebuilds.
      COMMAND "${CMAKE_COMMAND}" -E rm -f "${view}" "${view}.json"
      COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=${ROBODYNA_SOURCE_ROOT}"
              "${Python3_EXECUTABLE}" "${generator}"
              --root "${ROBODYNA_SOURCE_ROOT}" --ledger "${ledger}"
              --original-path "${original_path}"
              --expected-original-sha256 "${original_sha}"
              --expected-ledger-sha256 "${ledger_sha}" --output "${view}"
      DEPENDS "${generator}" "${helper}" "${ledger}" "${contract}"
              "${ROBODYNA_SOURCE_ROOT}/${canonical_relative}"
              "${ROBODYNA_SOURCE_ROOT}/${forwarder_relative}"
      VERBATIM
      COMMENT "Authenticate canonical ${family} and generate parser-only declarations")
    add_custom_target(${view_target} DEPENDS "${view}" "${view}.json")
    set_property(GLOBAL APPEND PROPERTY ROBODYNA_SWIG_DECLARATION_TARGETS ${view_target})
    set_property(GLOBAL APPEND PROPERTY ROBODYNA_SWIG_DECLARATION_FILES "${view}" "${view}.json")
  endif()
  add_dependencies(${target} ${view_target})
  if(TARGET ${target}_swig_compilation)
    add_dependencies(${target}_swig_compilation ${view_target})
  endif()
  set_property(TARGET ${target} APPEND PROPERTY SWIG_DEPENDS "${view}")
endfunction()

function(robodyna_bind_swig_declarations target)
  # These belong to SWIG's action, never the native C++ include closure.
  set_property(TARGET ${target} APPEND PROPERTY SWIG_INCLUDE_DIRECTORIES
               "${ROBODYNA_SOURCE_ROOT}/include" "${PROJECT_BINARY_DIR}")
  _robodyna_bind_swig_view(${target} body
    "docs/migration/BODY_TRANSFORMATIONS.json"
    "include/robodyna/mbd/RbBody.h"
    "src/compatibility/chrono/src/chrono/physics/ChBody.h")
  if(CH_ENABLE_MODULE_FEA)
    _robodyna_bind_swig_view(${target} mesh
      "docs/migration/MESH_TRANSFORMATIONS.json"
      "include/robodyna/fea/RbMesh.h"
      "src/compatibility/chrono/src/chrono/fea/ChMesh.h")
  endif()
endfunction()
