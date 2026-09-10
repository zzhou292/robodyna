include_guard(GLOBAL)
get_filename_component(robo_assembly_dynamics_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
if(NOT TARGET tl_explicit_nodal_state)
  message(FATAL_ERROR "Assembly dynamics requires the composing build's TL nodal owner")
endif()
include("${robo_assembly_dynamics_root}/case/source_assembly/SourceAssemblyWallSetup.cmake")
include("${robo_assembly_dynamics_root}/benchmarks/stage_timing/StageTiming.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/NativeRotation.cmake")
include("${robo_assembly_dynamics_root}/case/source_assembly_observation/SourceAssemblyObservation.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/qeph/QephBatch.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/t3/T3Batch.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/elements/ShellBatchPublication.cmake")
include("${ROBO_DYNA_TL_ROOT}/lib_src/collision/NodalWallContactDevice.cmake")
include("${robo_assembly_dynamics_root}/output/source_assembly/SourceAssemblyAcceptedOutput.cmake")
add_library(robo_dyna_source_assembly_dynamics STATIC
  "${CMAKE_CURRENT_LIST_DIR}/Case.cpp" "${CMAKE_CURRENT_LIST_DIR}/Config.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Startup.cpp" "${CMAKE_CURRENT_LIST_DIR}/Step.cu"
  "${CMAKE_CURRENT_LIST_DIR}/Checks.cpp" "${CMAKE_CURRENT_LIST_DIR}/ContactChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/OrientationChecks.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/ForceStage.cpp" "${CMAKE_CURRENT_LIST_DIR}/QephSpin.cpp")
target_link_libraries(robo_dyna_source_assembly_dynamics PUBLIC
  robo_dyna_source_native_rotation robo_dyna_stage_timing robo_dyna_source_assembly_wall_setup robo_dyna_source_assembly_observation
  robo_dyna_source_assembly_accepted_output tl_shell_batch_publication tl_nodal_wall_contact_device)
target_compile_features(robo_dyna_source_assembly_dynamics PUBLIC cxx_std_17)
set_target_properties(robo_dyna_source_assembly_dynamics PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED YES)
target_compile_options(robo_dyna_source_assembly_dynamics PRIVATE
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-fast-math;-ffp-contract=off>"
  "$<$<COMPILE_LANGUAGE:CUDA>:--fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false;-Xcompiler=-fno-fast-math,-ffp-contract=off>")
