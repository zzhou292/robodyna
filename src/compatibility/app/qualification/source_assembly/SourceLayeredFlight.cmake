set(ROBO_DYNA_SOURCE_ELASTIC_INVENTORY "" CACHE FILEPATH "Frozen original elastic arm V3 source")
set(ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY "" CACHE FILEPATH "Frozen original elastic/analytic/table V3 source")
if(ROBO_DYNA_SOURCE_ELASTIC_INVENTORY OR ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY)
  foreach(fixture ROBO_DYNA_SOURCE_ELASTIC_INVENTORY ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY)
    if(NOT EXISTS "${${fixture}}")
      message(FATAL_ERROR "Layered source flight requires explicit ${fixture}")
    endif()
  endforeach()
  target_sources(robo_dyna_source_assembly_flight_check PRIVATE
    SourceLayeredFlightTest.cpp SourceLayeredValues.cpp SourceLayeredParentChecks.cpp)
  set_property(TEST source_assembly_flight APPEND PROPERTY ENVIRONMENT
    "ROBO_DYNA_SOURCE_ELASTIC_INVENTORY=${ROBO_DYNA_SOURCE_ELASTIC_INVENTORY}"
    "ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY=${ROBO_DYNA_SOURCE_SECTION_MIXED_INVENTORY}")
endif()
