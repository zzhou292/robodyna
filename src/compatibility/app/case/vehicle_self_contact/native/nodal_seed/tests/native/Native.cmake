include_guard(GLOBAL)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  message(FATAL_ERROR "LAW42 contact-slot reference requires explicit GNU Fortran")
endif()
if(NOT ROBO_DYNA_TL_ROOT)
  message(FATAL_ERROR "LAW42 contact-slot reference needs the pinned TL helper root")
endif()
set(law42_slots_source "${CMAKE_CURRENT_LIST_DIR}")
set(law42_slots_generated "${CMAKE_CURRENT_BINARY_DIR}/law42-contact-slots-native")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${law42_slots_source}/prepare.py"
  --tl-root "${ROBO_DYNA_TL_ROOT}" --output "${law42_slots_generated}"
  COMMAND_ERROR_IS_FATAL ANY)
file(MAKE_DIRECTORY "${law42_slots_generated}/modules")
add_library(robo_dyna_law42_contact_slots_native STATIC
  "${law42_slots_generated}/Constants.F90" "${law42_slots_generated}/ContactSlots.F")
set_target_properties(robo_dyna_law42_contact_slots_native PROPERTIES
  Fortran_MODULE_DIRECTORY "${law42_slots_generated}/modules")
target_include_directories(robo_dyna_law42_contact_slots_native PUBLIC "${law42_slots_source}"
  PRIVATE "${law42_slots_generated}" "${law42_slots_generated}/modules")
target_compile_options(robo_dyna_law42_contact_slots_native PRIVATE
  -cpp -ffixed-line-length-none -ffree-line-length-none -fno-fast-math -ffp-contract=off -fcheck=bounds)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${law42_slots_source}/prepare.py" "${law42_slots_source}/ContactSlots.F"
  "${law42_slots_source}/source-manifest.json"
  "${ROBO_DYNA_TL_ROOT}/lib_utest/qualification/radioss_type25_selection/native/Sources.py")
add_test(NAME law42_contact_slots_source COMMAND "${Python3_EXECUTABLE}" -B
  "${law42_slots_source}/prepare.py" --tl-root "${ROBO_DYNA_TL_ROOT}"
  --output "${law42_slots_generated}" --check)
