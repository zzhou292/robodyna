include_guard(GLOBAL)
# Reuse the owning private native target; do not compile a second donor core.
function(add_qeph_projection_native target_name)
  if(NOT TARGET qeph_q1_native)
    message(FATAL_ERROR "Add qualification/native/qeph first")
  endif()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  get_filename_component(packet_root "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/.." ABSOLUTE)
  set(captured "${CMAKE_CURRENT_BINARY_DIR}/CapturedProjectionPackets.cpp")
  execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${packet_root}/fixtures/prepare.py"
    --output "${captured}" RESULT_VARIABLE code ERROR_VARIABLE error)
  if(NOT code EQUAL 0)
    message(FATAL_ERROR "Pinned projection packet failed admission: ${error}")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${packet_root}/fixtures/prepare.py" "${packet_root}/fixtures/manifest.json"
    "${packet_root}/fixtures/native-structural.jsonl")
  # Inherit that target's exact module/COMMON preprocessing, MVSIZ and flags.
  target_sources(qeph_q1_native PRIVATE "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/ProjectionReplay.F")
  add_library(${target_name} STATIC "${packet_root}/NativeReplay.cpp" "${captured}")
  target_compile_features(${target_name} PUBLIC cxx_std_17)
  target_compile_options(${target_name} PRIVATE -fno-fast-math -ffp-contract=off)
  target_include_directories(${target_name} PUBLIC "${packet_root}")
  target_link_libraries(${target_name} PUBLIC qeph_q1_native)
  add_test(NAME qeph_projection_packet_source COMMAND "${Python3_EXECUTABLE}" -B
    "${packet_root}/fixtures/prepare.py" --output "${captured}" --check)
  set_tests_properties(qeph_projection_packet_source PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
endfunction()

add_qeph_projection_native(qeph_projection_replay)
