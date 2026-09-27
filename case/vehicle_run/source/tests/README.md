# Pinned original input I/O extraction

This host-only package checks the narrow production reader extraction from app
17836638. The complete ReadOriginal, ReadCanonical and resolution bodies and
source constants are unchanged. Full-file reversals also keep the legacy source
constructor, field order and case composition unchanged. No native CLI or new
source graph is implemented here.

Five small GTests reuse the existing temporary-file helper and a literal frozen
ReadOriginal body. They cover exact binary/empty/chunk-sized bytes, missing and
oversized inputs, short/wrong-digest inputs, fresh reauthentication after file
replacement, and the genuine canonical-member rejection before file reads.
The unchanged real OriginalSources constructor is compiled as an object against
the narrowed headers. No original full model or GPU initialization is constructed. Resolution success
continues to rely on its unchanged existing producer and the literal body proof;
this package does not claim a new full-source resolution qualification.

CheckProduction.cmake inspects the product target dependency closure before test
targets are introduced. Configure with ROBO_ORIGINAL_SOURCE_IO_TESTS=OFF and
CMAKE_DISABLE_FIND_PACKAGE_GTest=TRUE to demonstrate the production library is
independent of GoogleTest. Existing source-mapping tests are disabled explicitly;
no test fixture is linked to the product library.

Run only after the root releases the workstation lane. Source proof success is
not a compiler, full-source, numerical or performance result.
