#pragma once

#include "elements/ReissnerShellData.h"

namespace chrono::fea {
class ChElementShellReissner4;
}

namespace crash::reference {

// Call immediately after the caller's single normal ChSystem::Setup(), before
// changing current frames. Copy implementation-sensitive, currently public
// Chrono rest fields into compact TL data; never repeat Setup or recapture the
// reference. Chrono retains ownership of the source element and its material.
//
// Admits an initially planar rectangle (aspect ratio <= 100) and exactly one
// centered, isotropic, linear elastic layer, with positive density/thickness.
// Arbitrary common initial rigid orientation and distinct initial nodal frame
// parameterizations are supported. Plasticity, damping, layers, offsets and
// other initial element geometry are rejected explicitly.
//
// Invalid/unsupported data throws std::invalid_argument, leaving BOTH caller
// outputs unchanged. Extraction and source mutation must be externally
// serialized. The source is not altered. The caller must not mutate prepared
// TL reference/section data after publication.
void CopyReissnerShellSetup(chrono::fea::ChElementShellReissner4& element,
                           tl::fea::reissner::ShellReference& reference,
                           tl::fea::reissner::ElasticSection& section);

}  // namespace crash::reference
