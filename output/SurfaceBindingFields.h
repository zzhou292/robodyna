#pragma once
#include "ArtifactIO.h"
namespace crash::visual { struct Binding; }
namespace crash::output {
// Pure source/topology serialization shared by accepted shell archives.
void AppendSurfaceBinding(Document&, const visual::Binding&);
}
