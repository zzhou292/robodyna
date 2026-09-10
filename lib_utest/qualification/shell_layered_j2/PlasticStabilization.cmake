# Additive adapter around the existing hash-verified native QEPH target.
get_target_property(qeph_native_modules qeph_q1_native Fortran_MODULE_DIRECTORY)
get_target_property(qeph_native_build qeph_q1_native BINARY_DIR)
get_filename_component(qeph_native_original "${CMAKE_CURRENT_LIST_DIR}/../native/qeph/original" ABSOLUTE)
add_library(qeph_plastic_stabilization_native STATIC "${CMAKE_CURRENT_LIST_DIR}/NativePlasticStabilization.F")
target_link_libraries(qeph_plastic_stabilization_native PUBLIC qeph_q1_native)
target_include_directories(qeph_plastic_stabilization_native PRIVATE
  "${qeph_native_modules}" "${qeph_native_build}/prepared/original/engine/share/spe_inc"
  "${qeph_native_original}/engine/share/spe_inc"
  "${qeph_native_original}/engine/share/includes" "${qeph_native_original}/engine/share/r8")
target_compile_options(qeph_plastic_stabilization_native PRIVATE
  -cpp -ffixed-line-length-none -fcheck=bounds -fbacktrace -fno-fast-math -ffp-contract=off)
target_compile_definitions(qeph_plastic_stabilization_native PRIVATE MYREAL8 CPP_mach=CPP_p4linux964 COMP_GFORTRAN)
add_executable(qeph_plastic_stabilization_check "${CMAKE_CURRENT_LIST_DIR}/PlasticStabilizationTest.cpp")
target_link_libraries(qeph_plastic_stabilization_check PRIVATE qeph_plastic_stabilization_native GTest::gtest_main)
target_compile_options(qeph_plastic_stabilization_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME qeph_plastic_stabilization_check COMMAND qeph_plastic_stabilization_check)
set_tests_properties(qeph_plastic_stabilization_check PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30)
