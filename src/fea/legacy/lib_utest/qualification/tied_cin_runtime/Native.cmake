include_guard(GLOBAL)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(TL_CIN_NATIVE_CACHE "" CACHE PATH "Authenticated openradioss-tied-interface-1 donor cache")
if(NOT TL_CIN_NATIVE_CACHE)
  message(FATAL_ERROR "TL_CIN_NATIVE_CACHE must name the authenticated existing source cache")
endif()
set(cin_native "${CMAKE_CURRENT_LIST_DIR}/native")
set(cin_generated "${CMAKE_CURRENT_BINARY_DIR}/cin-native-sources")
execute_process(COMMAND "${Python3_EXECUTABLE}" -B "${cin_native}/stage_sources.py"
  "${TL_ROOT}" "${TL_CIN_NATIVE_CACHE}" "${cin_generated}" COMMAND_ERROR_IS_FATAL ANY)
add_library(tied_cin_runtime_native STATIC
  "${cin_native}/Context.F90" "${cin_native}/Interfaces.F90"
  "${cin_native}/Packet.F90" "${cin_native}/Seed.F90" "${cin_native}/Witness.F90"
  "${cin_generated}/FullForce.F" "${cin_generated}/FullMotion.F"
  "${cin_generated}/FullDiagnostics.F" "${cin_generated}/FullWitness.F")
set_target_properties(tied_cin_runtime_native PROPERTIES
  Fortran_MODULE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/cin-native-modules")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/cin-native-modules")
target_include_directories(tied_cin_runtime_native PRIVATE "${cin_native}/context"
  "${cin_generated}" "${CMAKE_CURRENT_BINARY_DIR}/cin-native-modules")
target_compile_options(tied_cin_runtime_native PRIVATE -cpp -ffixed-line-length-none
  -ffree-line-length-none -fno-fast-math -ffp-contract=off -fcheck=all -finit-real=snan)
add_test(NAME tied_cin_runtime_source_identity COMMAND "${Python3_EXECUTABLE}" -B
  "${cin_native}/stage_sources.py" "${TL_ROOT}" "${TL_CIN_NATIVE_CACHE}" "${cin_generated}")
