include_guard(GLOBAL)
enable_language(Fortran)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "Whole native initial inventory requires the pinned GNU Fortran compiler")
endif()
set(initial_inventory_dir "${CMAKE_CURRENT_BINARY_DIR}/initial-inventory-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_LIST_DIR}/prepare.py"
  --tl-root "${TYPE25_INITIAL_SOURCE_ROOT}" --output "${initial_inventory_dir}" RESULT_VARIABLE initial_inventory_result)
if(NOT initial_inventory_result EQUAL 0)
  message(FATAL_ERROR "Independent initial inventory source preparation failed")
endif()
file(GLOB initial_inventory_donors CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/original/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${initial_inventory_donors}
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" "${CMAKE_CURRENT_LIST_DIR}/source-manifest.json"
  "${TYPE25_INITIAL_SOURCE_ROOT}/lib_utest/qualification/radioss_type25_selection/native/Sources.py"
  "${CMAKE_CURRENT_LIST_DIR}/Boundary.F90" "${CMAKE_CURRENT_LIST_DIR}/Wrapper.F90"
  "${CMAKE_CURRENT_LIST_DIR}/FullHistory.F90")
set(initial_inventory_generated Constants.F90 Boundary.F90 Tri7box.F SmallMargin.F90 Sort.c
  I25BUC_VOX1.F INSOL25.F I25TRIVOX1.F I25STO.F I25S1S2.F I25COR3T.F I25PEN3A.F PREPARE_INT25.F Wrapper.F90)
list(TRANSFORM initial_inventory_generated PREPEND "${initial_inventory_dir}/")
add_library(type25_initial_inventory_native STATIC "${CMAKE_CURRENT_LIST_DIR}/../NativeOracle.cpp" ${initial_inventory_generated})
target_compile_features(type25_initial_inventory_native PUBLIC cxx_std_17)
set_target_properties(type25_initial_inventory_native PROPERTIES Fortran_MODULE_DIRECTORY "${initial_inventory_dir}")
target_include_directories(type25_initial_inventory_native PRIVATE "${initial_inventory_dir}" PUBLIC "${TYPE25_INITIAL_SOURCE_ROOT}")
target_compile_options(type25_initial_inventory_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffixed-line-length-none;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
add_test(NAME type25_initial_inventory_source COMMAND "${Python3_EXECUTABLE}" -B
  "${CMAKE_CURRENT_LIST_DIR}/prepare.py" --tl-root "${TYPE25_INITIAL_SOURCE_ROOT}" --output "${initial_inventory_dir}" --check)
add_library(type25_initial_source_history_native STATIC "${CMAKE_CURRENT_LIST_DIR}/../FullHistory.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/FullHistory.F90")
target_link_libraries(type25_initial_source_history_native PUBLIC type25_initial_state_native)
target_include_directories(type25_initial_source_history_native PRIVATE "${initial_native_dir}")
target_compile_features(type25_initial_source_history_native PUBLIC cxx_std_17)
target_compile_options(type25_initial_source_history_native PRIVATE
  "$<$<COMPILE_LANGUAGE:Fortran>:-cpp;-ffree-line-length-none;-fcheck=bounds;-fbacktrace;-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>")
