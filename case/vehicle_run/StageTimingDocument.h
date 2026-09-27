#pragma once
#include "case/vehicle_dynamics/StepTiming.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_run::detail {
// Shared host-observation document; total and last-attempt counters are distinct.
output::Document StageTimingDocument(const vehicle_dynamics::StepTimingSnapshot&);
} // namespace crash::cases::vehicle_run::detail
