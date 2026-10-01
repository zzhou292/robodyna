# Generated declarations are SWIG inputs only, never native class definitions.
include_guard(GLOBAL)

function(robodyna_bind_swig_body target)
  if(NOT TARGET robodyna_swig_body_declarations)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    set(contract "${ROBODYNA_SOURCE_ROOT}/build_defs/bindings/body_view_contract.json")
    # Pin values become command arguments at configure time. Regenerate those
    # arguments when the reviewed contract changes, before rebuilding the view.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${contract}")
    file(READ "${contract}" contract_json)
    string(JSON original_path GET "${contract_json}" original_path)
    string(JSON original_sha GET "${contract_json}" expected_original_sha256)
    string(JSON ledger_sha GET "${contract_json}" expected_ledger_sha256)
    set(view_dir "${PROJECT_BINARY_DIR}/robodyna_swig")
    set(view "${view_dir}/BodyDeclarations.h")
    set(ledger "${ROBODYNA_SOURCE_ROOT}/docs/migration/BODY_TRANSFORMATIONS.json")
    set(generator "${ROBODYNA_SOURCE_ROOT}/tools/bindings/declaration_view.py")
    set(helper "${ROBODYNA_SOURCE_ROOT}/tools/migration/source_transform.py")
    file(MAKE_DIRECTORY "${view_dir}")
    add_custom_command(
      OUTPUT "${view}" "${view}.json"
      # Only these owned build outputs are replaced during an incremental build.
      COMMAND "${CMAKE_COMMAND}" -E rm -f "${view}" "${view}.json"
      COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=${ROBODYNA_SOURCE_ROOT}"
              "${Python3_EXECUTABLE}" "${generator}"
              --root "${ROBODYNA_SOURCE_ROOT}" --ledger "${ledger}"
              --original-path "${original_path}"
              --expected-original-sha256 "${original_sha}"
              --expected-ledger-sha256 "${ledger_sha}" --output "${view}"
      DEPENDS "${generator}" "${helper}" "${ledger}" "${contract}"
              "${ROBODYNA_SOURCE_ROOT}/include/robodyna/mbd/RbBody.h"
              "${ROBODYNA_SOURCE_ROOT}/src/compatibility/chrono/src/chrono/physics/ChBody.h"
      VERBATIM
      COMMENT "Authenticate canonical body and generate parser-only declarations")
    add_custom_target(robodyna_swig_body_declarations DEPENDS "${view}" "${view}.json")
  endif()
  set(view "${PROJECT_BINARY_DIR}/robodyna_swig/BodyDeclarations.h")
  add_dependencies(${target} robodyna_swig_body_declarations)
  if(TARGET ${target}_swig_compilation)
    add_dependencies(${target}_swig_compilation robodyna_swig_body_declarations)
  endif()
  # These properties belong to SWIG's own action, not the native include closure.
  set_property(TARGET ${target} APPEND PROPERTY SWIG_INCLUDE_DIRECTORIES
               "${ROBODYNA_SOURCE_ROOT}/include" "${PROJECT_BINARY_DIR}")
  set_property(TARGET ${target} APPEND PROPERTY SWIG_DEPENDS "${view}")
endfunction()
