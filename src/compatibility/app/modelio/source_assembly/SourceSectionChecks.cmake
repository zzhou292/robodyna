include_guard(GLOBAL)
set(ROBO_DYNA_SOURCE_ELASTIC_INVENTORY "" CACHE FILEPATH "Optional original elastic arm V3 inventory")
set(ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY "" CACHE FILEPATH "Optional original three-law V3 inventory")
if(ROBO_DYNA_SOURCE_ELASTIC_INVENTORY)
  foreach(path ROBO_DYNA_SOURCE_ELASTIC_INVENTORY ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY
      ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY)
    if(NOT EXISTS "${${path}}")
      message(FATAL_ERROR "Layered source checks require the explicit ${path}")
    endif()
  endforeach()
  # Existing wall value utilities use the established TL include-root name.
  set(CRASH_TL_FEA_SOURCE_DIR "${ROBO_DYNA_TL_ROOT}")
  include("${CMAKE_CURRENT_LIST_DIR}/../../case/source_assembly/SourceAssemblyWallSetup.cmake")
  add_executable(robo_dyna_source_section_input_check
    "${CMAKE_CURRENT_LIST_DIR}/tests/SourceSectionInputTest.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tests/SourceGlobalLaw1Test.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tests/SourceSectionRejectionTest.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tests/SourceSectionAuxiliaryTest.cpp")
  target_link_libraries(robo_dyna_source_section_input_check PRIVATE
    robo_dyna_source_assembly_wall_setup GTest::gtest_main)
  target_compile_options(robo_dyna_source_section_input_check PRIVATE -fno-fast-math -ffp-contract=off)
  add_test(NAME source_section_input COMMAND robo_dyna_source_section_input_check)
  set_tests_properties(source_section_input PROPERTIES RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 120
    ENVIRONMENT "ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=${ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY};ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY=${ROBO_DYNA_SOURCE_ANALYTIC_INVENTORY};ROBO_DYNA_SOURCE_ELASTIC_INVENTORY=${ROBO_DYNA_SOURCE_ELASTIC_INVENTORY};ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY=${ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY}")
endif()
