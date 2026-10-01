# Generated declarations are SWIG inputs only, never native class definitions.
include_guard(GLOBAL)

function(_robodyna_bind_swig_view target family contract_relative ledger_relative canonical_relative forwarder_relative)
  set(view_target "robodyna_swig_${family}_declarations")
  set(contract "${ROBODYNA_SOURCE_ROOT}/${contract_relative}")
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
    # The dependency edge must name the same canonical source that the generator
    # will authenticate from its ledger. Pins themselves remain action-time
    # checks, preserving rejection/cleanup on a changed copied contract.
    if(NOT original_path STREQUAL forwarder_relative)
      message(FATAL_ERROR "Registry forwarder differs from the declaration contract")
    endif()
    file(READ "${ledger}" ledger_json)
    string(JSON entry_count LENGTH "${ledger_json}" files)
    math(EXPR entry_last "${entry_count} - 1")
    set(found_entry FALSE)
    foreach(entry_index RANGE 0 ${entry_last})
      string(JSON entry_original GET "${ledger_json}" files ${entry_index} original_path)
      if(entry_original STREQUAL original_path)
        string(JSON entry_canonical GET "${ledger_json}" files ${entry_index} canonical_path)
        if(NOT entry_canonical STREQUAL canonical_relative OR found_entry)
          message(FATAL_ERROR "Registry canonical dependency differs from the source ledger")
        endif()
        set(found_entry TRUE)
      endif()
    endforeach()
    if(NOT found_entry)
      message(FATAL_ERROR "Registry original header is absent from the source ledger")
    endif()
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
  # Parser inputs only; none of these directories joins the native C++ closure.
  set_property(TARGET ${target} APPEND PROPERTY SWIG_INCLUDE_DIRECTORIES
               "${ROBODYNA_SOURCE_ROOT}/include" "${PROJECT_BINARY_DIR}")
  set(registry "${ROBODYNA_SOURCE_ROOT}/build_defs/bindings/declaration_views.json")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${registry}")
  file(READ "${registry}" registry_json)
  string(JSON schema GET "${registry_json}" schema)
  if(NOT schema STREQUAL "robodyna.swig_declaration_views.v1")
    message(FATAL_ERROR "Unsupported parser declaration registry")
  endif()
  string(JSON count LENGTH "${registry_json}" views)
  if(count LESS 1)
    message(FATAL_ERROR "Parser declaration registry must not be empty")
  endif()
  math(EXPR last "${count} - 1")
  foreach(index RANGE 0 ${last})
    foreach(key name contract ledger canonical forwarder requires_fea)
      string(JSON ${key} GET "${registry_json}" views ${index} ${key})
    endforeach()
    if(requires_fea AND NOT CH_ENABLE_MODULE_FEA)
      continue()
    endif()
    _robodyna_bind_swig_view(${target} "${name}" "${contract}" "${ledger}" "${canonical}" "${forwarder}")
  endforeach()
endfunction()
