include("${CRASH_TL_FEA_SOURCE_DIR}/lib_src/collision/Q4PlanarContact.cmake")
add_library(robo_dyna_guided_plate_case STATIC GuidedPlateCase.cpp GuidedPlateCaseSupport.cpp)
target_link_libraries(robo_dyna_guided_plate_case PUBLIC
  robo_dyna_guided_plate_admission crash_canonical_wall tl_reissner_shell_batch
  tl_q4_planar_contact crash_nodal_mesh_output)
target_link_libraries(robo_dyna_guided_plate_case PUBLIC robo_dyna_wall_tessellation)
target_compile_options(robo_dyna_guided_plate_case PRIVATE -fno-fast-math -ffp-contract=off)

add_executable(robo_dyna_guided_plate_check guided_plate_case_check.cpp)
target_link_libraries(robo_dyna_guided_plate_check PRIVATE robo_dyna_guided_plate_case GTest::gtest)
target_compile_options(robo_dyna_guided_plate_check PRIVATE -fno-fast-math -ffp-contract=off)
add_test(NAME guided_plate_smoke COMMAND robo_dyna_guided_plate_check "${CRASH_CANONICAL_WALL}")
set_tests_properties(guided_plate_smoke PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE PROCESSORS 1
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;MKL_NUM_THREADS=1")
