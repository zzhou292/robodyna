# Shared canonical-format array I/O. No full-shell schema, tests, case, solver,
# or AcceptedReplay dependency is needed to consume this target.
include("${CMAKE_CURRENT_LIST_DIR}/../ArtifactIO.cmake")
add_library(robo_dyna_bounded_array_io STATIC ../BoundedArrayIO.cpp ../BoundedArrayDescriptor.cpp)
target_link_libraries(robo_dyna_bounded_array_io PUBLIC robo_dyna_artifact_io)
target_compile_features(robo_dyna_bounded_array_io PUBLIC cxx_std_17)
target_compile_options(robo_dyna_bounded_array_io PRIVATE -fno-fast-math -ffp-contract=off)
