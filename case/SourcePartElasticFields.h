#pragma once
#include "source_part_elastic/SourcePartElasticCase.h"
#include "chrono/AcceptedSurfaceMesh.h"
#include "output/ArtifactIO.h"

namespace crash::cases::source_part_elastic {
visual::Binding SourcePartSurfaceBinding(const SourcePartElasticCase&, std::uint64_t run_id, std::uint64_t topology_id);
output::Document SourcePartConfiguration(const SourcePartElasticCase&, const visual::Binding&,
                                        std::uint64_t steps, unsigned frame_every);
output::Document SourcePartFrameFields(const Snapshot&);
void AppendSourcePartDiagnostics(output::Document&, const Diagnostics&);
} // namespace crash::cases::source_part_elastic
