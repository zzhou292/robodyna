#pragma once
#include "source_part_elastic/SourcePartElasticCase.h"
#include "output/ArtifactIO.h"
namespace crash::cases::source_part_elastic {
// Shared source identity and raw phase encoding; no experiment or loading policy.
void CheckSourcePartPhase(const Snapshot&,const std::array<double,3>& startup_velocity);
void AppendSourcePartKinematics(output::Document&,const Snapshot&);
void AppendSourcePartInputTables(output::Document&,const SourcePartElasticCase&);
}
