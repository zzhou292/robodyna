#pragma once
#include "SourceAssemblySurface.h"
#include "output/ArtifactIO.h"
#include "output/AcceptedReplay.h"
#include "lib_src/elements/ShellBatchPlasticity.h"

namespace crash::output::assembly {
struct SectionView {
    const tl::fea::ShellBatchSectionState* values=nullptr;
    const double* reported_thickness_m=nullptr;
    std::size_t count=0;
};
// Pure formatting/view validation: these views do not prove acceptance. The
// live-owner SourceAssemblyAcceptedOutput supplies accepted views. Raw host
// contract tests may supply clearly labelled synthetic values. Exact active
// counts are checked before reads. All three thickness points are preserved.
bool ValidSections(const SourceAssemblySurface&, SectionView qeph, SectionView t3) noexcept;
// Caller allocates exactly one scalar per source parent at startup. Validation
// finishes before any scalar changes; no allocation or energy reconstruction.
bool CopyParentScalars(const SourceAssemblySurface&, SectionView qeph, SectionView t3,
                       std::vector<ReplayParentScalar>&) noexcept;
// A field document, not an admitted replay-bundle schema or an acceptance token.
// Constructs privately and returns complete output, leaving caller state intact
// on any validation/allocation failure. Native work diagnostics remain named
// diagnostics: no global energy or connected-constraint ledger is synthesized.
Document SectionFieldDocument(const SourceAssemblySurface&, SectionView qeph, SectionView t3);
} // namespace crash::output::assembly
