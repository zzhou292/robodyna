#pragma once
#include "output/ArtifactIO.h"
namespace crash::output { struct ReplayInfo; }
namespace crash::visual { class AcceptedReplayScene; }
namespace crash::viewer {
// Capture metadata only; this does not modify the accepted input archive.
void AppendReplayColorMetadata(output::Document&, const output::ReplayInfo&,
                               const visual::AcceptedReplayScene&);
}
