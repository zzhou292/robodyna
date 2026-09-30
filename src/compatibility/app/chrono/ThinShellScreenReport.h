#pragma once

#include "ThinShellScreen.h"
#include "output/ArtifactIO.h"

namespace crash::reference {
// Serializes diagnostics, including rejected cases. Provenance is a bounded
// caller-supplied source/build inventory; it does not alter screen parameters.
// No report can claim simulation readiness. All numeric fields must be finite.
output::Document ThinShellScreenReport(const ThinShellScreenDiagnostic&,
                                      const output::Document& provenance);
} // namespace crash::reference
